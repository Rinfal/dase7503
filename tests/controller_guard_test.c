#include <assert.h>
#include "guard.h"

int main(void) {
    drive_guard_t g={0};
    int zero[4]={0},move[4]={800,800,-800,-800},bad[4]={801,0,0,0};
    assert(!guard_arm(&g,1000));
    assert(!guard_command(&g,move,2000));
    assert(g.duty[0]==0);
    assert(guard_command(&g,zero,3000));
    assert(guard_arm(&g,4000));
    assert(guard_command(&g,move,5000));
    guard_tick(&g,254999); assert(g.armed);
    guard_tick(&g,255000); assert(!g.armed && g.timed_out && g.duty[0]==0);
    /* New incoming motion cannot re-arm after a link timeout. */
    assert(!guard_command(&g,move,256000));
    assert(!guard_arm(&g,257000));
    assert(guard_command(&g,zero,258000));
    assert(guard_arm(&g,259000));
    assert(!guard_command(&g,bad,260000)); assert(!g.armed && g.duty[0]==0);
    assert(guard_command(&g,zero,261000));
    assert(!guard_arm(&g,500000));
    assert(guard_command(&g,zero,501000)); assert(guard_arm(&g,502000));
    assert(guard_command(&g,move,503000)); guard_stop(&g);
    for(int i=0;i<4;i++) assert(g.duty[i]==0);
    int negative_bad[4]={0,0,0,-801};
    assert(!guard_command(&g,negative_bad,504000));
    return 0;
}
