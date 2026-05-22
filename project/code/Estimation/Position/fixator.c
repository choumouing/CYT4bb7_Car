#include "fixator.h"

#define FIXATOR_SOURCE_NONE  (0U)
#define FIXATOR_SOURCE_ENTER (1U)
#define FIXATOR_SOURCE_TRACK (2U)

#define FIXATOR_EVENT_STRICT_RADIUS_M      (0.35f)
#define FIXATOR_EVENT_LOOSE_RADIUS_M       (0.90f)
#define FIXATOR_EVENT_MIN_SCORE            (0.50f)
#define FIXATOR_EVENT_MIN_CONFIDENCE       (1.00f)
#define FIXATOR_TRANSITION_RADIUS_M        (1.10f)
#define FIXATOR_TRANSITION_WEAK_RADIUS_M   (0.40f)
#define FIXATOR_TRACK_ACCEPT_RADIUS_M      (0.14f)
#define FIXATOR_TRACK_ACCEPT_MIN_SCORE     (0.55f)
#define FIXATOR_TRACK_ACCEPT_MIN_ABS_Z     (0.35f)
#define FIXATOR_TRACK_MIN_SEPARATION_TICK  (650U)
#define FIXATOR_INITIAL_B7_TRACK_RADIUS_M  (0.07f)
#define FIXATOR_INITIAL_B7_MIN_SCORE       (0.60f)

#define FIXATOR_ALPHA_INITIAL      (0.75f)
#define FIXATOR_ALPHA_TRANSITION   (0.50f)
#define FIXATOR_ALPHA_TRACK        (0.00f)
#define FIXATOR_ALPHA_WEAK         (0.00f)
#define FIXATOR_ALPHA_STRICT       (0.75f)
#define FIXATOR_ALPHA_REPEAT_ENTER (0.50f)

typedef enum
{
    FIXATOR_REASON_NONE = 0,
    FIXATOR_REASON_INITIAL,
    FIXATOR_REASON_STRICT,
    FIXATOR_REASON_TRANSITION,
    FIXATOR_REASON_WEAK,
    FIXATOR_REASON_TRACK,
    FIXATOR_REASON_REPEAT
} fixator_reason_t;

typedef struct
{
    uint16 beacon_index;
    float distance_m;
    float rank;
    fixator_reason_t reason;
} fixator_candidate_t;

fixator_data_t g_fixator = {0};

static uint32 s_last_enter_count;
static uint32 s_last_fix_tick;
static uint16 s_last_accepted_index;
static uint8 s_last_accepted_source;
static uint16 s_track_confirmed_mask;

static float fixator_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float fixator_distance2(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;

    return (dx * dx) + (dy * dy);
}

static uint8 fixator_get_beacon(uint16 index, beacon_config_point_t *beacon)
{
    if(index >= beacon_config_get_count())
    {
        return 0U;
    }

    return beacon_config_get_beacon(index, beacon);
}

static float fixator_distance_to_beacon(uint16 index, const float position[2])
{
    beacon_config_point_t beacon;

    if(fixator_get_beacon(index, &beacon) == 0U)
    {
        return 9999.0f;
    }

    return sqrtf(fixator_distance2(position[x], position[y], beacon.x, beacon.y));
}

static uint8 fixator_topology_allows(uint16 previous, uint16 candidate)
{
    if(previous == FIXATOR_NO_BEACON_INDEX)
    {
        return 1U;
    }

    switch(previous)
    {
        case 1U:    /* 2 号 -> 3,6 */
            return ((candidate == 2U) || (candidate == 5U)) ? 1U : 0U;
        case 2U:    /* 3 号 -> 2 */
            return (candidate == 1U) ? 1U : 0U;
        case 3U:    /* 4 号 -> 3,4 */
            return ((candidate == 2U) || (candidate == 3U)) ? 1U : 0U;
        case 4U:    /* 5 号 -> 4,5 */
            return ((candidate == 3U) || (candidate == 4U)) ? 1U : 0U;
        case 5U:    /* 6 号 -> 3,4,5,7 */
            return ((candidate == 2U) || (candidate == 3U) || (candidate == 4U) || (candidate == 6U)) ? 1U : 0U;
        case 6U:    /* 7 号 -> 6 */
            return (candidate == 5U) ? 1U : 0U;
        default:
            return 0U;
    }
}

static uint8 fixator_enter_allows(uint16 previous, uint16 candidate)
{
    if(previous == FIXATOR_NO_BEACON_INDEX)
    {
        return 1U;
    }

    if(candidate == previous)
    {
        return 1U;
    }

    return fixator_topology_allows(previous, candidate);
}

static uint8 fixator_initial_loose_allowed(uint16 candidate, float distance_m)
{
    uint8 location = (uint8)g_beacon_detection.location;
    float score = g_beacon_detection.score;

    if(candidate == 1U)
    {
        return ((location == 4U) && (distance_m <= 0.70f) && (score >= 1.20f)) ? 1U : 0U;
    }
    if(candidate == 2U)
    {
        return ((location == 2U) && (distance_m <= 0.90f) && (score >= 1.00f)) ? 1U : 0U;
    }
    if(candidate == 3U)
    {
        return ((location == 2U) && (distance_m <= 0.90f) && (score >= 1.00f)) ? 1U : 0U;
    }
    if(candidate == 5U)
    {
        return ((location == 3U) && (distance_m <= 0.80f) && (score >= 1.00f)) ? 1U : 0U;
    }
    if(candidate == 6U)
    {
        return ((distance_m <= 0.50f) && (score >= 1.00f)) ? 1U : 0U;
    }

    return 0U;
}

static float fixator_candidate_rank(uint16 candidate, float distance_m, const float position[2], uint16 previous)
{
    float rank = distance_m;

    if(previous == 5U)
    {
        if((candidate == 3U) && (position[y] < 2.80f))
        {
            rank -= 0.20f;
        }
        if((candidate == 2U) && (position[y] >= 2.80f))
        {
            rank -= 0.20f;
        }
        if((candidate == 6U) && (position[x] > 4.50f))
        {
            rank -= 0.25f;
        }
    }
    if((previous == 3U) && (candidate == 2U))
    {
        rank -= 0.15f;
    }
    if((previous == 3U) && (candidate == 3U))
    {
        rank += 0.35f;
    }
    if((previous == 4U) && (candidate == 4U))
    {
        rank += 0.20f;
    }
    if((previous == 2U) && (candidate == 1U))
    {
        rank -= 0.20f;
    }
    if((previous == 1U) && (candidate == 5U))
    {
        rank -= 0.20f;
    }

    return rank;
}

static float fixator_alpha(fixator_reason_t reason)
{
    switch(reason)
    {
        case FIXATOR_REASON_INITIAL:
            return FIXATOR_ALPHA_INITIAL;
        case FIXATOR_REASON_STRICT:
            return FIXATOR_ALPHA_STRICT;
        case FIXATOR_REASON_TRANSITION:
            return FIXATOR_ALPHA_TRANSITION;
        case FIXATOR_REASON_WEAK:
            return FIXATOR_ALPHA_WEAK;
        case FIXATOR_REASON_TRACK:
            return FIXATOR_ALPHA_TRACK;
        case FIXATOR_REASON_REPEAT:
            return FIXATOR_ALPHA_REPEAT_ENTER;
        default:
            return 0.0f;
    }
}

static void fixator_find_nearest(const float position[2], uint16 nearest[3], float nearest_dist[3])
{
    uint16 count = beacon_config_get_count();
    uint16 i;

    nearest[0] = FIXATOR_NO_BEACON_INDEX;
    nearest[1] = FIXATOR_NO_BEACON_INDEX;
    nearest[2] = FIXATOR_NO_BEACON_INDEX;
    nearest_dist[0] = 9999.0f;
    nearest_dist[1] = 9999.0f;
    nearest_dist[2] = 9999.0f;

    for(i = 0U; i < count; i++)
    {
        float distance_m = fixator_distance_to_beacon(i, position);

        if(distance_m < nearest_dist[0])
        {
            nearest_dist[2] = nearest_dist[1];
            nearest[2] = nearest[1];
            nearest_dist[1] = nearest_dist[0];
            nearest[1] = nearest[0];
            nearest_dist[0] = distance_m;
            nearest[0] = i;
        }
        else if(distance_m < nearest_dist[1])
        {
            nearest_dist[2] = nearest_dist[1];
            nearest[2] = nearest[1];
            nearest_dist[1] = distance_m;
            nearest[1] = i;
        }
        else if(distance_m < nearest_dist[2])
        {
            nearest_dist[2] = distance_m;
            nearest[2] = i;
        }
    }
}

static void fixator_clear_match_snapshot(void)
{
    g_fixator.last_match_valid = 0U;
    g_fixator.counts_in_sequence = 0U;
    g_fixator.fix_source = FIXATOR_SOURCE_NONE;
    g_fixator.beacon_index = FIXATOR_NO_BEACON_INDEX;
    g_fixator.before_position[x] = g_odometer.position[x];
    g_fixator.before_position[y] = g_odometer.position[y];
    g_fixator.fixed_position[x] = g_odometer.position[x];
    g_fixator.fixed_position[y] = g_odometer.position[y];
    g_fixator.match_distance2_m2 = 0.0f;
}

static void fixator_apply_candidate(uint16 beacon_index,
                                    fixator_reason_t reason,
                                    uint8 source,
                                    uint8 counts_in_sequence,
                                    float distance_m)
{
    beacon_config_point_t beacon;
    float alpha;

    if((g_fixator.pending_fix != 0U) || (fixator_get_beacon(beacon_index, &beacon) == 0U))
    {
        return;
    }

    alpha = fixator_alpha(reason);
    g_fixator.pending_fix = 1U;
    g_fixator.last_match_valid = 1U;
    g_fixator.counts_in_sequence = counts_in_sequence;
    g_fixator.fix_source = source;
    g_fixator.beacon_index = beacon_index;
    g_fixator.before_position[x] = g_odometer.position[x];
    g_fixator.before_position[y] = g_odometer.position[y];
    g_fixator.fixed_position[x] = g_odometer.position[x] + ((beacon.x - g_odometer.position[x]) * alpha);
    g_fixator.fixed_position[y] = g_odometer.position[y] + ((beacon.y - g_odometer.position[y]) * alpha);
    g_fixator.match_distance2_m2 = distance_m * distance_m;
    g_fixator.fix_count++;
    s_last_fix_tick = tick_1000us_cnt;
    s_last_accepted_index = beacon_index;
    s_last_accepted_source = source;

    if(counts_in_sequence != 0U)
    {
        g_fixator.previous_beacon_index = beacon_index;
        g_fixator.sequence_count++;
        if(source == FIXATOR_SOURCE_TRACK)
        {
            s_track_confirmed_mask |= (uint16)(1U << beacon_index);
        }
    }
}

static uint8 fixator_accept_enter_candidate(fixator_candidate_t *candidate)
{
    float position[2];
    uint16 nearest[3];
    float nearest_dist[3];
    uint16 previous = g_fixator.previous_beacon_index;
    uint16 count = beacon_config_get_count();
    uint16 i;
    uint8 found = 0U;
    fixator_candidate_t best;

    if(candidate == NULL)
    {
        return 0U;
    }

    if((float)g_beacon_detection.confidence < FIXATOR_EVENT_MIN_CONFIDENCE)
    {
        return 0U;
    }

    position[x] = g_odometer.position[x];
    position[y] = g_odometer.position[y];
    fixator_find_nearest(position, nearest, nearest_dist);

    best.beacon_index = FIXATOR_NO_BEACON_INDEX;
    best.distance_m = 9999.0f;
    best.rank = 9999.0f;
    best.reason = FIXATOR_REASON_NONE;

    for(i = 0U; i < count; i++)
    {
        float distance_m;
        fixator_reason_t reason = FIXATOR_REASON_NONE;

        if(fixator_enter_allows(previous, i) == 0U)
        {
            continue;
        }

        distance_m = fixator_distance_to_beacon(i, position);
        if((distance_m <= FIXATOR_EVENT_STRICT_RADIUS_M) &&
           (g_beacon_detection.score >= FIXATOR_EVENT_MIN_SCORE))
        {
            reason = FIXATOR_REASON_STRICT;
        }
        else if((previous != FIXATOR_NO_BEACON_INDEX) && (distance_m <= FIXATOR_TRANSITION_WEAK_RADIUS_M))
        {
            reason = FIXATOR_REASON_WEAK;
        }
        else if((previous != FIXATOR_NO_BEACON_INDEX) &&
                (distance_m <= FIXATOR_TRANSITION_RADIUS_M) &&
                (g_beacon_detection.score >= FIXATOR_EVENT_MIN_SCORE))
        {
            reason = FIXATOR_REASON_TRANSITION;
        }
        else if((previous == FIXATOR_NO_BEACON_INDEX) && (fixator_initial_loose_allowed(i, distance_m) != 0U))
        {
            reason = FIXATOR_REASON_INITIAL;
        }

        if(reason != FIXATOR_REASON_NONE)
        {
            float rank = fixator_candidate_rank(i, distance_m, position, previous);
            if((found == 0U) || (rank < best.rank))
            {
                best.beacon_index = i;
                best.distance_m = distance_m;
                best.rank = rank;
                best.reason = reason;
                found = 1U;
            }
        }
    }

    if(found == 0U)
    {
        (void)nearest;
        (void)nearest_dist;
        return 0U;
    }

    *candidate = best;
    return 1U;
}

static uint8 fixator_make_repeat_enter(fixator_candidate_t *candidate)
{
    float position[2];
    uint16 previous = g_fixator.previous_beacon_index;

    if(candidate == NULL)
    {
        return 0U;
    }

    if(previous != 3U)
    {
        return 0U;
    }
    if((uint8)g_beacon_detection.location != 4U)
    {
        return 0U;
    }
    if(g_beacon_detection.score < 1.0f)
    {
        return 0U;
    }

    position[x] = g_odometer.position[x];
    position[y] = g_odometer.position[y];
    candidate->beacon_index = previous;
    candidate->distance_m = fixator_distance_to_beacon(previous, position);
    candidate->rank = candidate->distance_m;
    candidate->reason = FIXATOR_REASON_REPEAT;

    return 1U;
}

static void fixator_handle_enter_event(void)
{
    fixator_candidate_t candidate;

    fixator_clear_match_snapshot();

    if(fixator_accept_enter_candidate(&candidate) != 0U)
    {
        uint8 duplicate_track_fix =
            ((s_last_accepted_source == FIXATOR_SOURCE_TRACK) &&
             (s_last_accepted_index == candidate.beacon_index)) ? 1U : 0U;
        uint8 duplicate_disallowed =
            ((s_last_accepted_index == candidate.beacon_index) &&
             (candidate.beacon_index != 3U) &&
             (candidate.beacon_index != 4U)) ? 1U : 0U;

        if((duplicate_track_fix == 0U) && (duplicate_disallowed == 0U))
        {
            fixator_apply_candidate(candidate.beacon_index,
                                    candidate.reason,
                                    FIXATOR_SOURCE_ENTER,
                                    1U,
                                    candidate.distance_m);
        }
    }
    else if(fixator_make_repeat_enter(&candidate) != 0U)
    {
        fixator_apply_candidate(candidate.beacon_index,
                                candidate.reason,
                                FIXATOR_SOURCE_ENTER,
                                0U,
                                candidate.distance_m);
    }
}

static uint8 fixator_track_candidate_allowed(uint16 beacon_index)
{
    uint32 dt;

    if(g_fixator.previous_beacon_index == FIXATOR_NO_BEACON_INDEX)
    {
        return 0U;
    }
    if(fixator_topology_allows(g_fixator.previous_beacon_index, beacon_index) == 0U)
    {
        return 0U;
    }
    dt = tick_1000us_cnt - s_last_fix_tick;
    if(dt <= FIXATOR_TRACK_MIN_SEPARATION_TICK)
    {
        return 0U;
    }
    if((s_last_accepted_index == beacon_index) && (beacon_index != 3U) && (beacon_index != 4U))
    {
        return 0U;
    }
    if((s_track_confirmed_mask & (uint16)(1U << beacon_index)) != 0U)
    {
        return 0U;
    }
    if((g_beacon_detection.score < FIXATOR_TRACK_ACCEPT_MIN_SCORE) &&
       (fixator_absf(g_beacon_detection.impact_robust_z) < FIXATOR_TRACK_ACCEPT_MIN_ABS_Z))
    {
        return 0U;
    }

    return 1U;
}

static void fixator_try_track_candidate(void)
{
    float position[2];
    uint16 previous = g_fixator.previous_beacon_index;
    uint16 count = beacon_config_get_count();
    uint16 i;
    uint8 found = 0U;
    fixator_candidate_t best;

    if(g_fixator.pending_fix != 0U)
    {
        return;
    }

    position[x] = g_odometer.position[x];
    position[y] = g_odometer.position[y];

    if(previous == FIXATOR_NO_BEACON_INDEX)
    {
        float distance_m = fixator_distance_to_beacon(6U, position);
        if((distance_m <= FIXATOR_INITIAL_B7_TRACK_RADIUS_M) &&
           (g_beacon_detection.score >= FIXATOR_INITIAL_B7_MIN_SCORE))
        {
            fixator_apply_candidate(6U,
                                    FIXATOR_REASON_TRACK,
                                    FIXATOR_SOURCE_TRACK,
                                    1U,
                                    distance_m);
        }
        return;
    }

    best.beacon_index = FIXATOR_NO_BEACON_INDEX;
    best.distance_m = 9999.0f;
    best.rank = 9999.0f;
    best.reason = FIXATOR_REASON_TRACK;

    for(i = 0U; i < count; i++)
    {
        float distance_m;
        float rank;

        if(fixator_topology_allows(previous, i) == 0U)
        {
            continue;
        }

        distance_m = fixator_distance_to_beacon(i, position);
        if(distance_m > FIXATOR_TRACK_ACCEPT_RADIUS_M)
        {
            continue;
        }

        rank = fixator_candidate_rank(i, distance_m, position, previous);
        if((found == 0U) || (rank < best.rank))
        {
            best.beacon_index = i;
            best.distance_m = distance_m;
            best.rank = rank;
            found = 1U;
        }
    }

    if((found != 0U) && (fixator_track_candidate_allowed(best.beacon_index) != 0U))
    {
        fixator_apply_candidate(best.beacon_index,
                                FIXATOR_REASON_TRACK,
                                FIXATOR_SOURCE_TRACK,
                                1U,
                                best.distance_m);
    }
}

void fixator_init(void)
{
    fixator_reset();
}

void fixator_reset(void)
{
    memset(&g_fixator, 0, sizeof(g_fixator));
    g_fixator.beacon_index = FIXATOR_NO_BEACON_INDEX;
    g_fixator.previous_beacon_index = FIXATOR_NO_BEACON_INDEX;
    s_last_enter_count = g_beacon_detection.enter_count;
    s_last_fix_tick = 0U;
    s_last_accepted_index = FIXATOR_NO_BEACON_INDEX;
    s_last_accepted_source = FIXATOR_SOURCE_NONE;
    s_track_confirmed_mask = 0U;
}

void fixator_update_100HZ(void)
{
    uint32 enter_count_now = g_beacon_detection.enter_count;

    if((g_beacon_detection.enter_event != 0U) && (enter_count_now != s_last_enter_count))
    {
        s_last_enter_count = enter_count_now;
        fixator_handle_enter_event();
    }

    fixator_try_track_candidate();
}

uint8 fixator_get_position_fix(float position[2])
{
    if((position == NULL) || (g_fixator.pending_fix == 0U))
    {
        return 0U;
    }

    position[x] = g_fixator.fixed_position[x];
    position[y] = g_fixator.fixed_position[y];
    g_fixator.pending_fix = 0U;

    return 1U;
}
