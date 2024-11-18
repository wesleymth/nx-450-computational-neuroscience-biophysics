#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;
#if defined(__cplusplus)
extern "C" {
#endif

extern void _VecStim_reg(void);
extern void _stdp_cc_reg(void);

void modl_reg() {
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");
    fprintf(stderr, " \"VecStim.mod\"");
    fprintf(stderr, " \"stdp_cc.mod\"");
    fprintf(stderr, "\n");
  }
  _VecStim_reg();
  _stdp_cc_reg();
}

#if defined(__cplusplus)
}
#endif
