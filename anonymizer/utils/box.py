import cv2

class Box:
    def __init__(self, x_min, y_min, x_max, y_max, score, kind):
        self.x_min = float(x_min)
        self.y_min = float(y_min)
        self.x_max = float(x_max)
        self.y_max = float(y_max)
        self.score = float(score)
        self.kind = str(kind)

    def __repr__(self):
        return f'Box({self.x_min}, {self.y_min}, {self.x_max}, {self.y_max}, {self.score}, {self.kind})'

    def __eq__(self, other):
        if isinstance(other, Box):
            return (self.x_min == other.x_min and self.y_min == other.y_min and
                    self.x_max == other.x_max and self.y_max == other.y_max and
                    self.score == other.score and self.kind == other.kind)
        return False


def draw_bbox(image, bbox):
    x_min, y_min, x_max, y_max = bbox
    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)


if __name__  == "__main__":
    image = cv2.imread("/home/jarvis/jw_ws/FusionPortable_utils/release/anonymizer/test/right.png")

    # bbox = [0, 620, 680, 768]
    
    # bbox = [0, 430, 430, 768]
    # bbox = [0,400,430,768]
    # '0,420,430,768','0,620,680,768'
    # ['0,430,430,768','300,650,1024,768']

    # ['0,400,430,768','0,620,680,768']
    for bbox in [[0,390,400,768], [0,620,680,768]]:
        draw_bbox(image, bbox)
    cv2.imwrite("/home/jarvis/jw_ws/FusionPortable_utils/release/anonymizer/test/right_test.png", image)