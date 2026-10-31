"""Drive one control chain cooperatively; grouping belongs to AsyncSim."""


class AsyncSimSession:
    def __init__(self, runtime, **kwargs):
        previous = getattr(runtime, "_driveable_session", None)
        if previous is not None and not previous.finished:
            raise RuntimeError("runtime already owns an active control session")
        self.runtime = runtime
        runtime._driveable_session = self
        self._steps = runtime._policy_steps(driveable=True, **kwargs)
        self.result = None
        self.finished = False
        self.phase = "created"

    def advance(self):
        if self.finished:
            return False
        try:
            self.phase = next(self._steps)
        except StopIteration as completed:
            self.result = completed.value
            self.finished = True
            self.phase = "finished"
            return False
        except BaseException:
            self.finished = True
            self.phase = "failed"
            raise
        return True

    def close(self):
        if not self.finished:
            try:
                self._steps.close()
            finally:
                self.finished = True
                self.phase = "closed"
