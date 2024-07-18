/* ************************************************************************
   FILE:		report.c
   COMMENT:		Evaluate the filter quality, i.e. calculate 
			in-band/out-of-band amplitude ripple, phase error ...
			This is not a C function, just peace of code to include
			if we want to generate the report after equiliser has
			been designed.
   CONTENT:
   AUTHOR:		Lime Microsystems
   DATE:		May 18, 2020
   REVISION:		Jul 02, 2020
   ************************************************************************ */

{
#ifdef VERBOSE
    // ---------------------------------------------------------------------------------------
    // Calculate amplitude, phase and group delay within the pass band for evaluation and plotting
    // ---------------------------------------------------------------------------------------
    void amphgd(int mode);
    amphgd(0);

    // ---------------------------------------------------------------------------------------
    // Optimization quality check
    // ---------------------------------------------------------------------------------------
    double ger, outgain, phierr;

    ger = phierr = 0.0;
    for (int i = 0; i < P; i++)
    {
        phierr += (phi[i] - phid[i]) * (phi[i] - phid[i]);
        ger += (a[i] - ad[i]) * (a[i] - ad[i]);
    }
    phierr /= (double)(P);
    ger /= (double)(P);

    double max = -100.0;
    for (int i = P; i < (P + Ps); i++)
        if (max < a[i])
            max = a[i];
    outgain = max;

    printf("%s OPTIMIZATION:\n", design);
    printf("    Initial error:           %lg\n", inierr);
    printf("    Final error:             %lg\n", finierr);
    printf("    Number of iterations:     %d\n", iter);
    printf("    Mean square gain error:  %lg\n", gerr);
    printf("    Mean square phase error: %lg rad^2\n", phierr);
    printf("    Out of band gain:        %lg\n", outgain);
    printf("    Beta:                    %lg Tclk\n", h[N]);

    // ---------------------------------------------------------------------------------------
    // Log data for plotting
    // ---------------------------------------------------------------------------------------
    char fname[100];
    FILE *fp;
    sprintf(fname, "%s_apg.txt", design);
    fp = fopen(fname, "w");
    if (fp == NULL)
    {
        printf("Can not open file %s.\n", fname);
        exit(0);
    }

    for (int i = 0; i < P; i++)
        fprintf(fp,
            "%lg %lg %lg %lg %lg %lg %lg %lg %lg %lg\n",
            x[i],
            ad[i],
            phid[i] / M_PI,
            taud[i],
            a[i],
            phi[i] / M_PI,
            tau[i] - h[N],
            a[i] - ad[i],
            phi[i] - phid[i],
            tau[i] - taud[i] - h[N]);
    fclose(fp);

    #if 0
	// ---------------------------------------------------------------------------------------
	// Write C include file
	// ---------------------------------------------------------------------------------------
	sprintf(fname, "%s.h", design); fp = fopen(fname, "w");
	if( fp == NULL ) { printf("Can not open file '%s'.\n", fname); exit(0);	}  
	
	fprintf(fp, "// ** Equilizer FIR filter **\n");
	fprintf(fp, "//    Sample clock frequency %g MHz, period %g ns\n", fclk, 1.0e3/fclk);
	fprintf(fp, "//    Number of taps %d\n", N);
	fprintf(fp, "//    Mean square gain error %lg\n", gerr);
	fprintf(fp, "//    Mean square phase error %lg rad**2\n", phierr);
	fprintf(fp, "//    Out of band gain: %lg\n", outgain);
	fprintf(fp, "//    Beta = %lg Tclk\n", h[N]);
	
	fprintf(fp, "int %s_n = %d;\n", FNAME, N);
	fprintf(fp, "double %s_h[] = {\n", FNAME);
	for(int i=0; i<N; i++) {
		if( i < (N-1) ) fprintf(fp, "\t%2.20lg,\n", h[i]);
		else fprintf(fp, "\t%2.20lg };\n", h[i]);
	}
	fclose(fp);
    #endif
#endif
}
