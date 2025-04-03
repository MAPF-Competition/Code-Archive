# Taken from https://gist.github.com/gmarkall/62308d951d4c1553ccbe608f391de46d
import itertools

import numba as nb
from numba.experimental import jitclass
from typing import List, Tuple, Dict
from heapq import heappush, heappop


# @jitclass
class PurePythonPriorityQueue:
    def __init__(self):
        self.pq = []  # list of entries arranged in a heap
        self.entry_finder = {}  # mapping of indices to entries
        self.REMOVED = -1  # placeholder for a removed item
        self.counter = itertools.count()  # unique sequence count

    def put(self, item: Tuple[int, int], priority: float = 0.0):
        """Add a new item or update the priority of an existing item"""
        if item in self.entry_finder:
            self.remove_item(item)
        count = next(self.counter)
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heappush(self.pq, entry)

    def remove_item(self, item: Tuple[int, int]):
        """Mark an existing item as REMOVED.  Raise KeyError if not found."""
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED

    def pop(self):
        """Remove and return the lowest priority item. Raise KeyError if empty."""
        while self.pq:
            priority, count, item = heappop(self.pq)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                return item
        raise KeyError("pop from an empty priority queue")


# @jitclass
# class Item:
#     i: int
#     j: int
#
#     def __init__(self, i, j):
#         self.i = i
#         self.j = j
#
#     def __eq__(self, other):
#         return self.i == other.i and self.j == other.j


@jitclass
class Entry:
    priority: float
    count: int
    item: Tuple[int, int]
    removed: bool

    def __init__(self, p: float, c: int, i: Tuple[int, int]):
        self.priority = p
        self.count = c
        self.item = i
        self.removed = False

    def __lt__(self, other):
        return self.priority < other.priority


LOWLEVELSIG = Tuple[Tuple[int, int], int, float, float, int]
HIGHLEVELSIG = Tuple[int, float, float, int]

@jitclass
class LowLevelEntry:
    priority: float
    count: int
    item: LOWLEVELSIG
    removed: bool

    def __init__(self, p: float, c: int, i: LOWLEVELSIG):
        self.priority = p
        self.count = c
        self.item = i
        self.removed = False

    def __lt__(self, other):
        return self.priority < other.priority


@jitclass
class HighLevelEntry:
    priority: float
    count: int
    item: HIGHLEVELSIG
    removed: bool

    def __init__(self, p: float, c: int, i: HIGHLEVELSIG):
        self.priority = p
        self.count = c
        self.item = i
        self.removed = False

    def __lt__(self, other):
        return self.priority < other.priority


@jitclass
class LowLevelPriorityQueue:
    pq: List[LowLevelEntry]
    entry_finder: Dict[LOWLEVELSIG, LowLevelEntry]
    counter: int

    def __init__(self):
        self.pq = nb.typed.List.empty_list(LowLevelEntry(0.0, 0, ((0, 0), 0, 0.0, 0.0, 0)))
        self.entry_finder = nb.typed.Dict.empty(((0, 0), 0, 0.0, 0.0, 0), LowLevelEntry(0.0, 0, ((0, 0), 0, 0.0, 0.0, 0)))
        self.counter = 0

    def put(self, item: LOWLEVELSIG, priority: float = 0.0):
        """Add a new item or update the priority of an existing item"""
        if item in self.entry_finder:
            self.remove_item(item)
        self.counter += 1
        entry = LowLevelEntry(priority, self.counter, item)
        self.entry_finder[item] = entry
        heappush(self.pq, entry)

    def remove_item(self, item: LOWLEVELSIG):
        """Mark an existing item as REMOVED.  Raise KeyError if not found."""
        entry = self.entry_finder.pop(item)
        entry.removed = True

    def pop(self) -> LOWLEVELSIG:
        """Remove and return the lowest priority item. Raise KeyError if empty."""
        while self.pq:
            #priority, count, item, removed = heappop(self.pq)
            entry = heappop(self.pq)
            if not entry.removed:
                del self.entry_finder[entry.item]
                return entry.item
        raise KeyError("pop from an empty priority queue")

    def empty(self):
        return len(self.pq) == 0


@jitclass
class HighLevelPriorityQueue:
    pq: List[HighLevelEntry]
    entry_finder: Dict[HIGHLEVELSIG, HighLevelEntry]
    counter: int

    def __init__(self):
        self.pq = nb.typed.List.empty_list(HighLevelEntry(0.0, 0, (0, 0.0, 0.0, 0)))
        self.entry_finder = nb.typed.Dict.empty((0.0, 0), HighLevelEntry(0.0, 0, (0, 0.0, 0.0, 0)))
        self.counter = 0

    def put(self, item: HIGHLEVELSIG, priority: float = 0.0):
        """Add a new item or update the priority of an existing item"""
        if item in self.entry_finder:
            self.remove_item(item)
        self.counter += 1
        entry = HighLevelEntry(priority, self.counter, item)
        self.entry_finder[item] = entry
        heappush(self.pq, entry)

    def remove_item(self, item: HIGHLEVELSIG):
        """Mark an existing item as REMOVED.  Raise KeyError if not found."""
        entry = self.entry_finder.pop(item)
        entry.removed = True

    def pop(self) -> HIGHLEVELSIG:
        """Remove and return the lowest priority item. Raise KeyError if empty."""
        while self.pq:
            #priority, count, item, removed = heappop(self.pq)
            entry = heappop(self.pq)
            if not entry.removed:
                del self.entry_finder[entry.item]
                return entry.item
        raise KeyError("pop from an empty priority queue")

    def empty(self):
        return len(self.pq) == 0

# if __name__ == "__main__":
#     queue1 = PurePythonPriorityQueue()
#     queue1.put((4, 5), 5.4)
#     queue1.put((5, 6), 1.0)
#     print(queue1.pop())  # Yay this works!
# 
#     queue2 = PriorityQueue()  # Nope
#     queue2.put((4, 5), 5.4)
#     queue2.put((5, 6), 1.0)
#     print(queue2.pop())
# 
#     # queue3 = OptimalQueue()
#     # for i in range(5):
#     #     for j in range(5):
#     #         queue3.add_index(i, j)
#     #         # [Index(i, j) for i in range(5) for j in range(5)]
#     # print(queue3.indices)
#     # queue3.put((4, 5), 5.4)
#     # queue3.put((5, 6), 1.0)
#     # print(queue3.pop())  # Yay this works!