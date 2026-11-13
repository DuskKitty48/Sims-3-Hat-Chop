/* Version 7: v6 (approved two-chop look + front-skin repair) plus hat fit:
 * non-hat accessories (circlets, ears, earmuffs) no longer chop, and the cut
 * is lifted to stay hidden inside each hat's own edge. See hat_chop_hat_fit.h. */
#define HC_PROFILE_BUILD 1
#define HC_FRONT_SKIN_GUARD 1
#define HC_HAT_FIT 1
#define HC_VERSION 7
#define HC_SUBVERSION 3     /* 7.1 children and toddlers; 7.2 full hats (TS4 Hats import): all hair hidden;
                               7.3 hair that reshaping would push into the face keeps its chop (front hair kept
                               as made, or cut only) + failed chops remembered (no lag on repeated rebuilds);
                               7.3 update: that fallback checks/restores only hair BELOW the hat edge, so locks under
                               the hat stay tucked in (no poking through a cap's front band) */
#include "hat_chop.c"
