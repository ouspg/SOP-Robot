import numpy as np
import cv2


# read class names from text file
classes = None
with open("./src/face_tracker/detection-model/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]


class Box:
    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

    def left(self):
        return int(self.x1)

    def top(self):
        return int(self.y1)

    def right(self):
        return int(self.x2)

    def bottom(self):
        return int(self.y2)


# function to get the output layer names
# in the architecture
def get_output_layers(net):

    layer_names = net.getLayerNames()

    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers


# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, class_id, x, y, x_plus_w, y_plus_h):

    label = str(classes[class_id])

    color = 255

    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)

    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def get_phone_boxes_from_image(image):

    Width = image.shape[1]
    Height = image.shape[0]
    scale = 0.00392

    # read pre-trained model and config file
    net = cv2.dnn.readNet("./src/face_tracker/detection-model/yolov4-tiny.weights", "./src/face_tracker/detection-model/yolov4-tiny.cfg")

    # create input blob
    blob = cv2.dnn.blobFromImage(image, scale, (416, 416), (0, 0, 0), True, crop=False)

    # set input blob for the network
    net.setInput(blob)

    # run inference through the network
    # and gather predictions from output layers
    outs = net.forward(get_output_layers(net))

    # initialization
    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4

    # for each detetion from each output layer
    # get the confidence, class id, bounding box params
    # and ignore weak detections (confidence < 0.5)
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and classes[class_id] == "cell phone":
                center_x = int(detection[0] * Width)
                center_y = int(detection[1] * Height)
                w = int(detection[2] * Width)
                h = int(detection[3] * Height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    # apply non-max suppression
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    # go through the detections remaining
    # after nms and draw bounding box
    box_objects = []
    for i in indices:
        box = boxes[i]
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]

        draw_bounding_box(
            image,
            class_ids[i],
            round(x),
            round(y),
            round(x + w),
            round(y + h),
        )

        box[2] = box[0] + w / 2
        box[3] = box[1] + h / 2
        box_objects.append({'left': int(box[0]), 'right': int(box[2]), 'top': int(box[1]), 'bottom': int(box[3]), 'face_id': ''})

    return box_objects


if __name__ == "__main__":
    image = cv2.imread("testi2.jpg")
    get_phone_boxes_from_image(image)
