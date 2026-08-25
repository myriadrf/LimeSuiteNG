#include "../common.h"
#include "args.hxx"

#include "limesuiteng/SDRDescriptor.h"
#include "comms/PCIe/LA9310_PCIe.h"
#include "chips/LA9310/PHYTimer.h"

#include <fcntl.h>
#include <stdio.h>

#include "chips/LA9310/vspa/VSPA_Trace.h"

#include <unordered_map>

extern void ToTraceFile(std::ofstream& ofs, const std::vector<l1_trace_data_t> data);

static const int max_trace_len = 400;

typedef enum {
    VSPA_MMAP_NONE = 0,
    VSPA_MMAP_L1_TRACE,
    VSPA_MMAP_ADC0,
    VSPA_MMAP_ADC1,
    VSPA_MMAP_ADC2,
    VSPA_MMAP_ADC3,
    VSPA_MMAP_RXDMA_LANE0,
    VSPA_MMAP_RXDMA_LANE1,
    VSPA_MMAP_RXDMA_LANE2,
    VSPA_MMAP_RXDMA_LANE3,
    VSPA_MMAP_DAC,
    VSPA_MMAP_TXDMA_LANE0,
    VSPA_MMAP_STATS,
    VSPA_MMAP_STATS2,
} e_vspa_feature;

#define QUOTE(name) #name

#define KeyValue(prefix, value) \
    { \
        prefix##value, "\"" QUOTE(value) "\"" \
    }

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

#define RX_NUM_CHAN 1

static std::atomic<bool> stopProgram(false);
static void intHandler(int dummy)
{
    stopProgram.store(true);
}

static void print_trace(const l1_trace_data_t* data, uint32_t length)
{
    printf("L1 TRACE (%u):\n", length);
    uint64_t base_cnt = data[0].cnt;
    for (uint32_t i = 0; i < length; ++i)
    {
        if (data[i].msg == 0)
            break;

        std::string cmdname; // = GetMsgName(data[i].msg);
        printf("%3u: [%+16li] %s %08X\n", i, data[i].cnt - base_cnt, cmdname.c_str(), data[i].param);
        base_cnt = data[i].cnt;
    }
}

static uint64_t event_count = 0;
static void DumpTracer(VSPA_Trace* tracer, std::ofstream& ofs)
{
    if (!tracer)
        return;
    auto events = tracer->ReadTrace();
    event_count += events.size();

    ToTraceFile(ofs, events);
}

// static vspa_state_t GetProxy(std::shared_ptr<LA9310_PCIe> pcie)
// {
//     vspa_state_t proxy;
//     auto iqflood = pcie->GetBar(LA9310_WINDOW_IQFLOOD);
//     if (iqflood.vaddr == nullptr)
//     {
//         printf("Failed proxy\n");
//         return proxy;
//     }

//     const uint32_t* v_vspa_dmem_proxy_ro = reinterpret_cast<const uint32_t*>(iqflood.vaddr);
//     // pcie->dmem_sync_to_cpu(v_vspa_dmem_proxy_ro, sizeof(vspa_state_t));
//     memcpy(&proxy, v_vspa_dmem_proxy_ro, sizeof(vspa_state_t));
//     // pcie->dmem_sync_to_device(v_vspa_dmem_proxy_ro, sizeof(vspa_state_t));
//     return proxy;
// }

// static void print_pipeline_tx(const tx_pipeline_t& pipe, const tx_pipeline_t& last_pipe)
// {
//     printf("Tx pipeline:\n");
//     uint32_t adc_rate = pipe.dac.output.bytes_done - last_pipe.dac.output.bytes_done;
//     printf("DAC | enq:%08X done:%08X, rate:%8u\n", pipe.dac.input.bytes_done, pipe.dac.output.bytes_done, adc_rate);
//     printf("INT | enq:%08X done:%08X\n", pipe.interp.input.bytes_done, pipe.interp.output.bytes_done);
//     uint32_t ddr_rate = pipe.ddr.output.bytes_done - last_pipe.ddr.output.bytes_done;
//     printf("DDR | enq:%08X done:%08X, rate:%8u\n", pipe.ddr.input.bytes_done, pipe.ddr.output.bytes_done, ddr_rate);
// }

// static void print_pipeline_rx(const rx_pipeline_t& pipe, const rx_pipeline_t& last_pipe)
// {
//     printf("Rx pipeline:\n");
//     uint32_t adc_rate = pipe.adc.output.bytes_done - last_pipe.adc.output.bytes_done;
//     printf("ADC | enq:%08X done:%08X, rate:%8u\n", pipe.adc.input.bytes_done, pipe.adc.output.bytes_done, adc_rate);
//     uint32_t ddr_rate = pipe.ddr.output.bytes_done - last_pipe.ddr.output.bytes_done;
//     printf("DDR | enq:%08X done:%08X, rate:%8u\n", pipe.ddr.input.bytes_done, pipe.ddr.output.bytes_done, ddr_rate);
// }

static void print_dma_bits(uint32_t bits)
{
    printf("\tW4:%i ", bool(bits & (1 << 15)));
    printf("W3:%i ", bool(bits & (1 << 14)));
    printf("W2:%i ", bool(bits & (1 << 13)));
    printf("W1:%i ", bool(bits & (1 << 12)));
    printf("DAC:%i ", bool(bits & (1 << 11)));
    printf("R4:%i ", bool(bits & (1 << 10)));
    printf("R3:%i ", bool(bits & (1 << 9)));
    printf("R2:%i ", bool(bits & (1 << 8)));
    printf("R1:%i ", bool(bits & (1 << 7)));
    printf("RSSI:%i ", bool(bits & (1 << 6)));
    printf("AUX:%i ", bool(bits & (1 << 5)));
    printf("RX1:%i ", bool(bits & (1 << 4)));
    printf("RX0:%i ", bool(bits & (1 << 3)));
    printf("R01:%i ", bool(bits & (1 << 2)));
    printf("R00:%i ", bool(bits & (1 << 1)));
    printf("W5:%i", bool(bits & (1 << 0)));
}

#define VSPA_CCSR 0x1000000
#define DMA_DMEM_PRAM_ADDR 0xB0
static void la9310_hexdump_dma(std::shared_ptr<LA9310_PCIe> pcie)
{
    auto bar0 = pcie->GetBar(LA9310_WINDOW_BAR0);
    if (bar0.vaddr == nullptr)
        return;

    volatile uint32_t* vals = reinterpret_cast<volatile uint32_t*>(uint64_t(bar0.vaddr) + VSPA_CCSR + DMA_DMEM_PRAM_ADDR);

    // const uint32_t* vals = reinterpret_cast<const uint32_t*>(ptr);
    vals += 4;
    printf("\nVSPA DMA regs (IP reg 0xB0):");
    printf("\nDMA_STAT_ABORT:\t%04X", *vals);
    print_dma_bits(*vals);
    vals++;
    printf("\nDMA_STAT_IRQ:\t%04X", *vals);
    print_dma_bits(*vals);
    vals++;
    printf("\nDMA_COMP_STATS:\t%04X", *vals);
    print_dma_bits(*vals);
    vals++;
    printf("\nDMA_XFERR_STAT:\t%04X", *vals);
    print_dma_bits(*vals);
    vals++;
    printf("\nDMA_CFGERR_STA:\t%04X", *vals);
    print_dma_bits(*vals);
    vals++;
    printf("\nDMA_XRUN_STAT:\t%04X", *vals);
    print_dma_bits(*vals);
    vals++;
    printf("\nDMA_GO_STAT:\t%04X", *vals);
    print_dma_bits(*vals);
    vals++;
    printf("\nDMA_FIFO_STAT:\t%04X", *vals);
    print_dma_bits(*vals);
    printf("\n");
}

static uint32_t GetValue32AtOffset(volatile void* base, uint32_t offset)
{
    auto ptr = reinterpret_cast<volatile uint8_t*>(base) + offset;
    return *reinterpret_cast<volatile uint32_t*>(ptr);
}

static void la9310_hexdump_control(std::shared_ptr<LA9310_PCIe> pcie)
{
    auto bar0 = pcie->GetBar(LA9310_WINDOW_BAR0);
    if (bar0.vaddr == nullptr)
        return;

    volatile uint32_t* base = reinterpret_cast<volatile uint32_t*>(uint64_t(bar0.vaddr) + VSPA_CCSR);

    printf("\nEvents:");
    printf("\nCONTROL:\t%08x ENTRY:\t%08x STACK:\t%08x",
        GetValue32AtOffset(base, 0x8),
        GetValue32AtOffset(base, 0x180),
        GetValue32AtOffset(base, 0x184));
    printf("\nSTATUS: \t%08x IRQEN:\t%08X", GetValue32AtOffset(base, 0x10), GetValue32AtOffset(base, 0xC));
    printf("\nEXT_GO_ENA:\t%08x EXT_GO_STAT:\t%08x", GetValue32AtOffset(base, 0x28), GetValue32AtOffset(base, 0x2C));
    printf("\nVCPU_HOST_FLAGS0_1:\t%08x \t%08x", GetValue32AtOffset(base, 0x14), GetValue32AtOffset(base, 0x18));
    printf("\nHOST_VCPU_FLAGS0_1:\t%08x \t%08x", GetValue32AtOffset(base, 0x1C), GetValue32AtOffset(base, 0x20));
    printf("\n");
}

static void la9310_dump_vspa_gp(std::shared_ptr<LA9310_PCIe> pcie)
{
    const uint32_t GP_IN0 = 0x500;
    const uint32_t GP_OUT0 = 0x580;

    auto bar0 = pcie->GetBar(LA9310_WINDOW_BAR0);
    if (bar0.vaddr == nullptr)
        return;

    printf("GPIN:\t");
    volatile uint32_t* gpin = reinterpret_cast<volatile uint32_t*>(uint64_t(bar0.vaddr) + VSPA_CCSR + GP_IN0);
    for (uint32_t i = 0; i <= 9; ++i)
        printf(" [%i]%08X", i, gpin[i]);
    printf("\n");
    uint32_t gpin0 = gpin[0];
    printf("RO0 - En:%i FnotEmpty:%i UDR:%i OVR:%i\n", gpin0 & 0x1, (gpin0 >> 1) & 1, (gpin0 >> 2) & 1, (gpin0 >> 3) & 1);
    gpin0 >>= 4;
    printf("RO1 - En:%i FnotEmpty:%i UDR:%i OVR:%i\n", gpin0 & 0x1, (gpin0 >> 1) & 1, (gpin0 >> 2) & 1, (gpin0 >> 3) & 1);
    gpin0 >>= 4;
    printf("RX0 - En:%i FnotEmpty:%i UDR:%i OVR:%i\n", gpin0 & 0x1, (gpin0 >> 1) & 1, (gpin0 >> 2) & 1, (gpin0 >> 3) & 1);
    gpin0 >>= 4;
    printf("RX1 - En:%i FnotEmpty:%i UDR:%i OVR:%i\n", gpin0 & 0x1, (gpin0 >> 1) & 1, (gpin0 >> 2) & 1, (gpin0 >> 3) & 1);
    uint32_t gpin1 = gpin[1];
    printf("TxCh5 - Cannot generate underflow: %8X\n", gpin1 & 0xFFFF);
    gpin1 >>= 16;
    printf("TxCh5 - En:%i FnotFull:%i UDR:%i OVR:%i\n", gpin1 & 0x1, (gpin1 >> 1) & 1, (gpin1 >> 2) & 1, (gpin1 >> 3) & 1);

    printf("\nGPOUT:\t");
    volatile uint32_t* gpout = reinterpret_cast<volatile uint32_t*>(uint64_t(bar0.vaddr) + VSPA_CCSR + GP_OUT0);
    for (uint32_t i = 0; i <= 9; ++i)
        printf(" [%i]%08X", i, gpout[i]);
    printf("\n");

    const uint32_t gpout7 = gpout[7];
    printf("Tx - En:%i Fthresh:%i ClrErr:%i\n", gpout7 & 0x1, (gpout7 >> 1) & 0x3, (gpout7 >> 4) & 1);
    uint32_t gpout4 = gpout[4];
    printf("RxCh1 - En:%i Fthresh:%i ClrErr:%i\n", gpout4 & 0x1, (gpout4 >> 1) & 0x3, (gpout4 >> 4) & 1);
    gpout4 >>= 8;
    printf("RxCh2 - En:%i Fthresh:%i ClrErr:%i\n", gpout4 & 0x1, (gpout4 >> 1) & 0x3, (gpout4 >> 4) & 1);
    gpout4 >>= 8;
    printf("RxCh3 - En:%i Fthresh:%i ClrErr:%i\n", gpout4 & 0x1, (gpout4 >> 1) & 0x3, (gpout4 >> 4) & 1);
    gpout4 >>= 8;
    printf("RxCh4 - En:%i Fthresh:%i ClrErr:%i\n", gpout4 & 0x1, (gpout4 >> 1) & 0x3, (gpout4 >> 4) & 1);
    printf("\n");
}

static void la9310_dump_dma(std::shared_ptr<LA9310_PCIe> pcie)
{
    // uint8_t* BAR2_addr = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR2).vaddr);
    // auto vspa_dmem_proxy_wo = reinterpret_cast<volatile vspa_state_t*>(BAR2_addr + 0x400000);

    // vspa_state_t* vspa_interface = const_cast<vspa_state_t*>(vspa_dmem_proxy_wo);
    // dma_table_t* table = &vspa_interface->internals.tx_dma_schedule;

    // uint32_t head = table->head;
    // uint32_t tail = table->tail;
    // printf("DMA: c:%3u p:%3u\n", head, tail);
    // auto row = table->items;
    // for (int i = 0; i < DMA_TABLE_LINE_COUNT; ++i)
    // {
    //     printf(i == (head & (DMA_TABLE_LINE_COUNT - 1)) ? "c" : " ");
    //     printf(i == (tail & (DMA_TABLE_LINE_COUNT - 1)) ? "p" : " ");
    //     printf("|addr:%08X sz:%8u phyt:%08X_%08X f:%04X\n",
    //         row[i].addr,
    //         row[i].size,
    //         row[i].timestamp >> 32,
    //         row[i].timestamp,
    //         row[i].flags);
    // }
}

typedef struct {
    uint32_t feature;
    uint32_t address;
} vspa_feature_t;

void* vspa_memorymap_find(std::shared_ptr<LA9310_PCIe> pcie, uint32_t f)
{
    uint8_t* base_addr = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR2).vaddr) + 0x400000;
    const vspa_feature_t* row = reinterpret_cast<const vspa_feature_t*>(base_addr);

    for (int i = 0; i < 8; ++i)
    {
        if (row[i].feature == 0)
            return NULL; // end of table

        if (row[i].feature == f)
            return (void*)(base_addr + (row[i].address << 1)); // row address is in VSPA halfwords, convert to bytes
    }
    return NULL;
}

struct ADC_lane {
    uint32_t base_buffer;
    uint32_t next_completion_buffer;
    uint32_t axi_fifo_addr;
    uint32_t completion_count;
    uint16_t axi_fifo_index;
    uint16_t dma_channel;
};

struct DebugStats {
    uint32_t adc_enq;
    uint32_t adc_compl;
    uint32_t ddr_enq;
    uint32_t ddr_compl;
    uint32_t ddr_ovr;
    uint32_t adc_err;
    uint32_t ddr_err;
};

struct DebugStats2 {
    uint32_t afe_enq;
    uint32_t afe_compl;
    uint32_t afe_err;
    uint32_t afe_udr;
    uint32_t dfe_enq;
    uint32_t dfe_compl;
    uint32_t dfe_udr;
    uint32_t dfe_err;
};

void dump_adc(ADC_lane* adc)
{
    printf("ADC:\n");
    printf("buf: \t%08x\n", adc->next_completion_buffer);
    printf("axi: \t%08x\n", adc->axi_fifo_addr);
    printf("axii:\t%08x\n", adc->axi_fifo_index);
    printf("dma:\t%08x\n", adc->dma_channel);
    printf("cmpl:\t%08x\n", adc->completion_count);
}

static auto stats_t1 = std::chrono::high_resolution_clock::now();
static auto stats2_t1 = std::chrono::high_resolution_clock::now();

#define RATE_OF(name, now, last, duration) (static_cast<double>(now->name - last->name) / duration)

void dump_stats(DebugStats* now, DebugStats* prev)
{
    auto stats_t2 = std::chrono::high_resolution_clock::now();
    double duration_s = std::chrono::duration_cast<chrono::milliseconds>(stats_t2 - stats_t1).count() / 1e3;
    stats_t1 = stats_t2;
    printf("adc_enq: \t%08x, %i/s\n", now->adc_enq, (int)RATE_OF(adc_enq, now, prev, duration_s));
    printf("adc_cmp: \t%08x, %i/s\n", now->adc_compl, (int)RATE_OF(adc_compl, now, prev, duration_s));
    printf("ddr_enq:\t%08x, %i/s\n", now->ddr_enq, (int)RATE_OF(ddr_enq, now, prev, duration_s));
    printf("ddr_cmp:\t%08x, %i/s\n", now->ddr_compl, (int)RATE_OF(ddr_compl, now, prev, duration_s));
    printf("ddr_ovr:\t%08x, %i/s\n", now->ddr_ovr, (int)RATE_OF(ddr_ovr, now, prev, duration_s));
    printf("adc_err:\t%08x, %i/s\n", now->adc_err, (int)RATE_OF(adc_err, now, prev, duration_s));
    printf("ddr_err:\t%08x, %i/s\n", now->ddr_err, (int)RATE_OF(ddr_err, now, prev, duration_s));
}

void dump_stats2(DebugStats2* now, DebugStats2* prev)
{
    auto stats_t2 = std::chrono::high_resolution_clock::now();
    double duration_s = std::chrono::duration_cast<chrono::milliseconds>(stats_t2 - stats2_t1).count() / 1e3;
    stats2_t1 = stats_t2;
    printf("afe_enq: \t%08x, %i/s\n", now->afe_enq, (int)RATE_OF(afe_enq, now, prev, duration_s));
    printf("afe_cmp: \t%08x, %i/s\n", now->afe_compl, (int)RATE_OF(afe_compl, now, prev, duration_s));
    printf("afe_err:\t%08x, %i/s\n", now->afe_err, (int)RATE_OF(afe_err, now, prev, duration_s));
    printf("afe_udr:\t%08x, %i/s\n", now->afe_udr, (int)RATE_OF(afe_udr, now, prev, duration_s));
    printf("dfe_enq:\t%08x, %i/s\n", now->dfe_enq, (int)RATE_OF(dfe_enq, now, prev, duration_s));
    printf("dfe_compl:\t%08x, %i/s\n", now->dfe_compl, (int)RATE_OF(dfe_compl, now, prev, duration_s));
    printf("dfe_udr:\t%08x, %i/s\n", now->dfe_udr, (int)RATE_OF(dfe_udr, now, prev, duration_s));
    printf("dfe_err:\t%08x, %i/s\n", now->dfe_err, (int)RATE_OF(dfe_err, now, prev, duration_s));
}

int main(int argc, char* argv[])
{
    args::ArgumentParser parser("limeVSPA - VSPA memory dispaly", "");
    args::HelpFlag help(parser, "help", "Display this help menu", { 'h', "help" });
    args::ValueFlag<std::string> devName(parser, "name", "Specifies which device to use", { 'd', "device" }, "");

    args::Flag trace(parser, "trace", "Event trace", { 't' });
    args::Flag gpio(parser, "gpio", "print VSPA GPIN GPOUT", { 'g' });
    args::Flag dma(parser, "dma", "print VSPA DMA", { 'x' });
    args::Flag ctrl(parser, "ctrl", "print Control", { 'e' });
    args::ValueFlag<int> channel(parser, "chan", "Which channel internals", { 'c', "channel" }, 0);

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return -1;
    }

    std::shared_ptr<LA9310_PCIe> pcie = std::make_shared<LA9310_PCIe>();
    OpStatus status = pcie->Open(args::get(devName), O_RDWR);

    if (status != OpStatus::Success)
        return EXIT_FAILURE;

    signal(SIGINT, intHandler);

    PHYTimer phytimer(pcie);

    // VSPA_iqplayer vspa(pcie);

    std::ofstream fout;
    if (trace)
    {
        fout.open("trace.json");
        fout << "{\n"
             << "\"displayTimeUnit\":\"ns\"," << "\"traceEvents\": [\n";

        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 100, \"args\": {"
                "\"name\" : \"DMA_WR_priority\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 101, \"args\": {"
                "\"name\" : \"ADC_RO0\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 102, \"args\": {"
                "\"name\" : \"VSPA_DMA\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 103, \"args\": {"
                "\"name\" : \"ADC_RX0\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 104, \"args\": {"
                "\"name\" : \"ADC_RX1\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 105, \"args\": {"
                "\"name\" : \"AUX_ADC\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 106, \"args\": {"
                "\"name\" : \"RSSI_RD\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 107, \"args\": {"
                "\"name\" : \"DDR_RD1\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 108, \"args\": {"
                "\"name\" : \"DDR_RD2\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 109, \"args\": {"
                "\"name\" : \"DDR_RD3\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 110, \"args\": {"
                "\"name\" : \"DDR_RD4\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 111, \"args\": {"
                "\"name\" : \"DAC\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 112, \"args\": {"
                "\"name\" : \"DDR_WR1\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 113, \"args\": {"
                "\"name\" : \"DDR_WR2 TRACE\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 114, \"args\": {"
                "\"name\" : \"DDR_WR3\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 115, \"args\": {"
                "\"name\" : \"DDR_WR4 State\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 1, \"args\": {"
                "\"name\" : \"VCPU\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 2, \"args\": {"
                "\"name\" : \"DMA\" }"
                "}\n";
        fout << "{\"name\": \"process_name\", \"ph\": \"M\", \"pid\": 3, \"args\": {"
                "\"name\" : \"IPPU\" }"
                "}\n";

        // fout.close();
    }

    ADC_lane* adc = reinterpret_cast<ADC_lane*>(vspa_memorymap_find(pcie, VSPA_MMAP_ADC0));
    ADC_lane* adc2 = nullptr;
    reinterpret_cast<ADC_lane*>(vspa_memorymap_find(pcie, VSPA_MMAP_ADC1));
    // DAC_lane* dac = reinterpret_cast<ADC_lane*>(vspa_memorymap_find(pcie, 12));
    DebugStats* stats = reinterpret_cast<DebugStats*>(vspa_memorymap_find(pcie, VSPA_MMAP_STATS));
    DebugStats2* stats2 = reinterpret_cast<DebugStats2*>(vspa_memorymap_find(pcie, VSPA_MMAP_STATS2));

    DebugStats last_stats;
    DebugStats2 last_stats2;

    while (stopProgram.load() == false)
    {
        if (trace)
        {
            // DumpTracer(vspa.tracer.get(), fout);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // std::cerr << "Evt: " << event_count << std::endl;
            // break;
        }
        else
        {
            printf("\033[2J");
            printf("\033[H");
            // proxy = GetProxy(pcie);
            // print_proxy(proxy, last_proxy, args::get(channel));
            la9310_hexdump_dma(pcie);
            if (gpio)
                la9310_dump_vspa_gp(pcie);

            if (dma)
                la9310_dump_dma(pcie);
            if (ctrl)
                la9310_hexdump_control(pcie);

            const std::array<uint8_t, 11> timer_ids = { 0, 1, 2, 3, 4, 10, 11, 12, 15, 20, 14 };
            std::cerr << "Timers:\n";
            for (auto id : timer_ids)
                std::cerr << phytimer.GetTimerControl(id).ToString() << std::endl;

            if (adc)
                dump_adc(adc);
            if (adc2)
                dump_adc(adc2);
            if (stats)
            {
                DebugStats temp = *stats;
                dump_stats(&temp, &last_stats);
                last_stats = temp;
            }
            if (stats2)
            {
                DebugStats2 temp = *stats2;
                dump_stats2(&temp, &last_stats2);
                last_stats2 = temp;
            }

            // phytimer.DumpMem();
            // RequestProxy(pcie);
            // last_proxy = proxy;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (trace)
    {
        fout << "]\n}";
        fout.close();
    }

    return EXIT_SUCCESS;
}
