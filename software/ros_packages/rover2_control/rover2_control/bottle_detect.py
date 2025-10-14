import tensorrt as trt
import cv2
import numpy as np
import time
import torch
import torchvision.transforms as transforms

class bottle_detector:
    def __init__(self):
        self.engine_path = "/home/makemorerobot/best.engine"  # Path to the TensorRT engine file
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.trt_logger = trt.Logger(trt.Logger.WARNING)

        # Load TensorRT runtime
        with open(self.engine_path, "rb") as f:
            runtime = trt.Runtime(self.trt_logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.input_shape = (1, 3, 640, 640)  # Adjust based on your YOLO model input size

    def preprocess_image(self, image):
        """Preprocess input image for TensorRT YOLO model"""
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((640, 640)),  # Resize to YOLO model input size
            transforms.ToTensor(),
            transforms.Normalize([0.0, 0.0, 0.0], [1.0, 1.0, 1.0])
        ])
        image = transform(image).unsqueeze(0).cuda()  # Add batch dimension
        return image

    def get_bottle(self, image):
        image_height, image_width, _ = image.shape  # Get height and width
        image_diagonal = np.sqrt(image_width**2 + image_height**2)  # Compute diagonal distance

        # Preprocess image
        img_tensor = self.preprocess_image(image)

        # Allocate buffers
        input_binding_idx = self.engine.get_binding_index("input")
        output_binding_idx = self.engine.get_binding_index("output")

        # Allocate CUDA buffers
        d_input = torch.cuda.FloatTensor(*self.input_shape).cuda()
        d_output = torch.cuda.FloatTensor(1, 25200, 85).cuda()  # Adjust shape for your model output

        bindings = [int(d_input.data_ptr()), int(d_output.data_ptr())]
        self.context.execute_v2(bindings)

        output = d_output.cpu().numpy()  # Move output to CPU

        # Post-processing
        results = self.process_detections(output, image_width, image_height, image_diagonal)

        if results:
            return results
        else:
            print("No water bottle detected.")
            return None, None

    def process_detections(self, output, image_width, image_height, image_diagonal):
        """Post-process the detections from the YOLO TensorRT output"""
        boxes, scores, class_ids = output[..., :4], output[..., 4], output[..., 5].astype(int)

        for i in range(len(boxes)):
            if class_ids[i] == 0 and scores[i] > 0.5:  # Assuming "WaterBottle" is class 0
                x_center, y_center, width, height = boxes[i]
                
                x_pct = x_center / float(image_width)
                diag_pct = np.sqrt(width**2 + height**2) / image_diagonal

                return x_pct, diag_pct

        return None

# Run detection
inst = bottle_detector()
image = cv2.imread("/home/makemorerobot/it.jpg")
x, diag = inst.get_bottle(image)

if x is not None:
    print(f"WaterBottle detected at: {x:.2f}, Diagonal %: {diag:.2f}")

