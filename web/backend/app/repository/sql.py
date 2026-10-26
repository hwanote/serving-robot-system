"""SQLAlchemy repository (SQLite now; any SQLAlchemy URL such as PostgreSQL later)."""
import json
import threading

from sqlalchemy import (Column, Integer, MetaData, String, Table, Text, create_engine, func,
                        insert, select, update)

from .base import Repository

TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED')


class SqlRepository(Repository):
    def __init__(self, url):
        args = {'check_same_thread': False} if url.startswith('sqlite') else {}
        self.engine = create_engine(url, connect_args=args, future=True)
        md = MetaData()
        self.tasks = Table(
            'tasks', md,
            Column('seq', Integer, primary_key=True, autoincrement=True),
            Column('task_id', String(32), unique=True, nullable=False),
            Column('client_request_id', String(128), unique=True),
            Column('status', String(16), nullable=False, index=True),
            Column('data', Text, nullable=False))
        self.events = Table(
            'events', md,
            Column('id', Integer, primary_key=True, autoincrement=True),
            Column('kind', String(32), nullable=False),
            Column('payload', Text, nullable=False))
        md.create_all(self.engine)
        self._lock = threading.Lock()

    @staticmethod
    def _row(r):
        return json.loads(r.data) if r is not None else None

    def next_task_id(self):
        with self._lock, self.engine.connect() as c:
            n = c.execute(select(func.coalesce(func.max(self.tasks.c.seq), 0))).scalar()
        # ids must stay unique even if rows were deleted: use the highest number seen
        return 'task-%04d' % (n + 1)

    def save_task(self, task):
        crid = task.get('client_request_id')
        with self._lock, self.engine.begin() as c:
            have = c.execute(select(self.tasks.c.seq).where(
                self.tasks.c.task_id == task['task_id'])).first()
            blob = json.dumps(task, ensure_ascii=False)
            if have:
                c.execute(update(self.tasks).where(self.tasks.c.task_id == task['task_id'])
                          .values(status=task['status'], data=blob))
            else:
                c.execute(insert(self.tasks).values(
                    task_id=task['task_id'], client_request_id=crid,
                    status=task['status'], data=blob))

    def get_task(self, task_id):
        with self.engine.connect() as c:
            return self._row(c.execute(select(self.tasks.c.data).where(
                self.tasks.c.task_id == task_id)).first())

    def _all(self, where=None, desc=False, limit=None):
        q = select(self.tasks.c.data)
        if where is not None:
            q = q.where(where)
        q = q.order_by(self.tasks.c.seq.desc() if desc else self.tasks.c.seq)
        if limit:
            q = q.limit(limit)
        with self.engine.connect() as c:
            return [json.loads(r.data) for r in c.execute(q)]

    def list_tasks(self):
        return self._all(desc=True)

    def recent_tasks(self, n):
        return list(reversed(self._all(desc=True, limit=n)))

    def find_by_request_id(self, client_request_id):
        rows = self._all(self.tasks.c.client_request_id == client_request_id)
        return rows[0] if rows else None

    def unfinished_tasks(self):
        return self._all(self.tasks.c.status.notin_(TERMINAL))

    def add_event(self, kind, payload):
        with self._lock, self.engine.begin() as c:
            c.execute(insert(self.events).values(
                kind=kind, payload=json.dumps(payload, ensure_ascii=False)))

    def recent_events(self, n):
        with self.engine.connect() as c:
            rows = c.execute(select(self.events.c.kind, self.events.c.payload)
                             .order_by(self.events.c.id.desc()).limit(n)).all()
        return [(r.kind, json.loads(r.payload)) for r in reversed(rows)]
