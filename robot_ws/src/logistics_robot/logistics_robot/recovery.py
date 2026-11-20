"""Select recovery actions from observed data, never from an active PID alone."""


class RecoveryPolicy:
    def __init__(self, grace=45.0, stale=15.0, cooldown=60.0):
        self.grace, self.stale, self.cooldown = grace, stale, cooldown
        self.last_restart = {}

    def choose(self, now, units, seen, web_ok, serial_present):
        def active(kind):
            state, started = units.get(kind, ('inactive', now))
            return state == 'active' and now-started >= self.grace

        def fresh(kind):
            return kind in seen and 0 <= now-seen[kind] < self.stale

        candidates = []
        if active('camera') and not fresh('camera'):
            candidates.append('camera')
        elif active('vision') and fresh('camera') and not fresh('vision'):
            candidates.append('vision')
        if active('agent') and serial_present and not fresh('agent'):
            candidates.append('agent')
        if active('remote') and not web_ok:
            candidates.append('remote')
        # Do not turn a persistent device fault into a restart storm. A manually
        # stopped service stays stopped until the operator starts the stack.
        selected = [kind for kind in candidates
                    if now-self.last_restart.get(kind, -float('inf')) >= self.cooldown]
        for kind in selected:
            self.last_restart[kind] = now
        return selected
