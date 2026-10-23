from .base import Repository


class MemoryRepository(Repository):
    def __init__(self):
        self._tasks, self._order, self._counter = {}, [], 0
        self.events = []

    def next_task_id(self):
        self._counter += 1
        return 'task-%04d' % self._counter

    def save_task(self, task):
        if task['task_id'] not in self._tasks:
            self._order.append(task['task_id'])
        self._tasks[task['task_id']] = task

    def get_task(self, task_id):
        return self._tasks.get(task_id)

    def list_tasks(self):
        return [dict(self._tasks[i]) for i in reversed(self._order)]

    def recent_tasks(self, n):
        return [dict(self._tasks[i]) for i in self._order[-n:]]

    def add_event(self, kind, payload):
        self.events = (self.events + [(kind, payload)])[-1000:]

    def find_by_request_id(self, client_request_id):
        for t in self._tasks.values():
            if t.get('client_request_id') == client_request_id:
                return dict(t)
        return None

    def unfinished_tasks(self):
        return [dict(self._tasks[i]) for i in self._order
                if self._tasks[i]['status'] not in ('SUCCEEDED', 'FAILED', 'CANCELED')]

    def recent_events(self, n):
        return list(self.events[-n:])
