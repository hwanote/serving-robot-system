class Repository:
    """Tasks and the event log. A task is a plain dict (see FleetManager.create_task)."""

    def next_task_id(self):
        raise NotImplementedError

    def save_task(self, task):
        raise NotImplementedError

    def get_task(self, task_id):
        raise NotImplementedError

    def list_tasks(self):
        """All tasks, newest first."""
        raise NotImplementedError

    def recent_tasks(self, n):
        """The last n tasks, oldest first."""
        raise NotImplementedError

    def add_event(self, kind, payload):
        raise NotImplementedError

    def find_by_request_id(self, client_request_id):
        """The task created with this client_request_id, or None."""
        raise NotImplementedError

    def unfinished_tasks(self):
        """Tasks not in a terminal state, oldest first (used to recover after a restart)."""
        raise NotImplementedError

    def recent_events(self, n):
        raise NotImplementedError
