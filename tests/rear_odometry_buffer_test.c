#include <assert.h>
#include <stdint.h>

#include "../code/rear_motor/rear_odometry_buffer.h"

int main(void)
{
    rear_odometry_buffer_t buffer;

    rear_odometry_buffer_init(&buffer);
    assert(rear_odometry_buffer_take(&buffer) == 0);

    rear_odometry_buffer_add(&buffer, 120, 300);
    rear_odometry_buffer_add(&buffer, 130, 300);
    assert(buffer.last_sample == 130);
    assert(buffer.pending_pulses == 250);
    assert(buffer.total_pulses == 250);
    assert(buffer.rejected_samples == 0);
    assert(buffer.max_abs_sample == 130);

    assert(rear_odometry_buffer_take(&buffer) == 250);
    assert(buffer.pending_pulses == 0);
    assert(buffer.total_pulses == 250);

    rear_odometry_buffer_add(&buffer, -80, 300);
    assert(rear_odometry_buffer_take(&buffer) == -80);
    assert(buffer.total_pulses == 170);

    rear_odometry_buffer_add(&buffer, 350, 300);
    assert(buffer.last_sample == 350);
    assert(buffer.pending_pulses == 0);
    assert(buffer.total_pulses == 170);
    assert(buffer.rejected_samples == 1);
    assert(buffer.rejected_pulses == 350);
    assert(buffer.max_abs_sample == 350);

    return 0;
}
