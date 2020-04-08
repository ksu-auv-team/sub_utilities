class Detection:
    def __init__(self, score, box, class_id):
        self.score = score
        self.box = box
        self.class_id = class_id
