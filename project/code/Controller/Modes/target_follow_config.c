#include "target_follow_config.h"


void target_follow_load_default_targets(void)
{
    target_follow_clear_targets();

    (void)target_follow_add_target(2.00f, 2.00f);
    (void)target_follow_add_target(-2.00f, -0.50f);
    (void)target_follow_add_target(0.00f, -1.50f);

    target_follow_restart_targets();
}
