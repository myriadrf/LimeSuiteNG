clear all;

LoadLimeSuiteNG

LimeInitialize();               % open first device

LimeLoadConfig('trxTest.ini');  % load configuration from file
                                % use LimeSuiteGUI to create configuration file

readCnt = 1024*64;          %samples to read (64K)
fifoSize = 1024*1024        %set library FIFO size to 1 MSample

LimeStartStreaming(fifoSize,["rx0"; "rx1"]); % start RX from channels 0 and 1

%receive samples, overwrite the same array
for i=1:40
    samples = LimeReceiveSamples(readCnt); % read samples from all RX channels
end
LimeGetStreamStatus()     %must run at least 1s to get data rate (B/s)
%stop streaming
LimeStopStreaming();      % stop streaming
LimeDestroy();            % close device
%plot samples
figure(1)
plot(samples(1,:));
figure(2)
plot(samples(2,:));

