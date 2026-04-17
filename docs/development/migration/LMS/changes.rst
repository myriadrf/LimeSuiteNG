LMS API changes
===============

This is the table noting all the known differences to the functionality of the legacy LMS API included in Lime Suite NG compared to the previous iteration of LimeSuite (version 23.11.0).

..
  TODO: Correct this whenever code changes

.. list-table:: Table 1: LMS API functionality changes
   :header-rows: 1

   * - LMS API function
     - Changes
   * - ``LMS_GetSampleRate()``
     - ``rf_Hz`` will always be the same as ``host_Hz`` (i.e. :math:`2^{ratio}` times smaller compared to the old API).
   * - ``LMS_SetLOFrequency()``
     - No functional difference once the bugs are fixed.
   * - ``LMS_GetLOFrequency()``
     - The NCO offset is currently not applied.
   * - ``LMS_SetLPF()``
     - Currently just sets the last saved LPF value from LMS_SetLPFBW call.
   * - ``LMS_SetSampleRateDir()``
     - Sets both Rx and Tx to the same sample rate.
   * - ``LMS_GetNCOFrequency()``
     - Negative values can be returned as well, use ``std::fabs()`` to fix that.
