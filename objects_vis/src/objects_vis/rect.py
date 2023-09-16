from dataclasses import dataclass


@dataclass
class Rect:
    x: int
    y: int
    w: int
    h: int

    def __and__(self, other):
        return self.intersection(other)

    def __or__(self, other):
        return self.union(other)

    def area(self):
        return self.w * self.h

    def intersection(self, other):
        x = max(self.x, other.x)
        y = max(self.y, other.y)
        w = min(self.x + self.w, other.x + other.w) - x
        h = min(self.y + self.h, other.y + other.h) - y
        if w <= 0 or h <= 0:
            return Rect(0, 0, 0, 0)
        return Rect(x, y, w, h)

    def union(self, other):
        x = min(self.x, other.x)
        y = min(self.y, other.y)
        w = max(self.x + self.w, other.x + other.w) - x
        h = max(self.y + self.h, other.y + other.h) - y
        return Rect(x, y, w, h)

    def tl(self):
        return int(self.x), int(self.y)

    def br(self):
        return self.x + self.w, self.y + self.h

    def slices(self):
        return slice(self.y, self.y + self.h), slice(self.x, self.x + self.w)