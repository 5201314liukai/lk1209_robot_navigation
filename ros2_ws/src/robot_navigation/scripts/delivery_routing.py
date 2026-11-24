"""Admission and exclusive routing for the shared TurtleBot3 House doorway."""
from collections import Counter
from dataclasses import dataclass


@dataclass(eq=False)
class Ticket:
    robot: str
    station: str


class SharedRoute:
    def __init__(self, robots):
        self.robots = list(robots)
        self.next_robot = 0
        self.queue = []
        self.active = None
        self.stranded = None

    def enqueue(self, requested, station, ready):
        robot = requested
        if not robot:
            loads = Counter(t.robot for t in self.queue)
            if self.active:
                loads[self.active.robot] += 1
            rotated = self.robots[self.next_robot:] + self.robots[:self.next_robot]
            available = [name for name in rotated if name in ready]
            robot = min(available or rotated, key=lambda name: loads[name])
            self.next_robot = (self.robots.index(robot) + 1) % len(self.robots)
        ticket = Ticket(robot, station)
        self.queue.append(ticket)
        return ticket

    def claim(self, ticket):
        if self.active:
            return False
        eligible = self.queue
        if self.stranded:
            eligible = [t for t in self.queue if t.robot == self.stranded and t.station == 'home']
        if not eligible or eligible[0] is not ticket:
            return False
        self.queue.remove(ticket)
        self.active = ticket
        return True

    def drop(self, ticket):
        self.queue.remove(ticket)

    def finish(self, ticket, at_home):
        assert self.active is ticket
        self.stranded = None if at_home else ticket.robot
        self.active = None
