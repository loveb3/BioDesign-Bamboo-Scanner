import serial
import os
import cv2
import time
import csv
from datetime import datetime
from collections import defaultdict


# Serial port settings
SERIAL_PORT = 'COM10'  # Change this to match your Arduino's serial port
BAUD_RATE = 115200


# imaging = False

width = 3264  # Specify the width in pixels
height = 2448  # Specify the height in pixels


# connect to serial port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Serial port {SERIAL_PORT} opened successfully.")
except serial.SerialException as e:
    print(f"Failed to open serial port {SERIAL_PORT}: {e}")


def get_line_from_serial_monitor():

    # reads serial port
    line = ser.readline().decode('utf-8').rstrip()
    print(f"Received from serial: '{line}'")

    # signal to take a picture
    if line == "P":
        imaging = True
        return "P", None, None, None
    #   signal to stop collecting data
    if line == "D":
        print("End of data transmission.")
        return "D", None, None, None

    try:
        # Splitting the received line by comma and stripping extra spaces
        parts = [part.strip() for part in line.split(',')]
        # Expecting exactly 4 parts for xPosition, angle, radius
        if len(parts) == 4:
            x_position, angle, radius, encoder_position = map(float, parts)  # Converting each part to float
            return x_position, angle, radius, encoder_position
        else:
            print("Error: Unexpected data format")
            return None, None, None, None
    except ValueError as e:
        print(f"Error processing line '{line}': {e}")
        return None, None, None, None

# Function to create a directory for the run if it doesn't exist
def create_directory():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(os.getcwd(), f"Run_{timestamp}")
    os.makedirs(path, exist_ok=True)
    return path
# Function to capture image from USB camera
def capture_image(cam_port, camera_num, folder_path):
    # Allow the camera to warm up for 3 seconds
    cam = cv2.VideoCapture(cam_port, cv2.CAP_DSHOW)

    # Set the desired width and height for the captured images
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Allow the camera to warm up for 1 second
    print(f"Warming up camera {camera_num}...")
    time.sleep(1)

    # capture 5 frames
    for i in range(5):
        # reading the input using the camera
        result, image = cam.read()

        # If image is detected without any error,
        if result:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_path = os.path.join(folder_path, f"camera{camera_num}_{timestamp}.png")

            # saving image in local storage
            if i == 2:
                # showing result
                cv2.imshow("Frame", image)
                cv2.imwrite(file_path, image)  # Ensure to use file_path including folder_path
                print(f"Image captured from camera {camera_num} saved at {file_path}")

            cv2.waitKey(30)

    # Destroy all windows
    cv2.destroyAllWindows()

def average_values(temp_sensor_data, folder_path):

    data_dict = defaultdict(list)

    # uses the x position and angle of rotation as the independent variables
    # if the x position and angle of rotation  are the same for any points the radius and encoder position are averaged
    for row in temp_sensor_data:
        key = (row[0], row[1])  # Use x_position and angle as the key
        data_dict[key].append((float(row[2]), float(row[3])))  # Append radius and encoder position

    # names the final csv file based on the current time
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(folder_path, f'final_data_{timestamp}.csv')

    # Process the data to average the radius for each (x position, angle) pair
    processed_data = [('X Position', 'Angle', 'Average Radius', 'Average Encoder Position')]
    for (x_position, angle), values in data_dict.items():
        average_radius = sum(value[0] for value in values) / len(values)
        average_encoder_position = sum(value[1] for value in values) / len(values)
        processed_data.append((x_position, angle, average_radius, average_encoder_position))

    # Write the processed data to a new CSV file
    with open(output_file, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(processed_data)
    print(f"Processed data has been saved to {output_file}")

# Main function
def main():

    # Creates a folder for the run
    folder_path = create_directory()  # Create a folder for this run
    # Temporary storage for sensor data
    temp_sensor_data = []

    # Depending on the line from the serial port, we either take a picture, stop collecting data,
    # or add the x position, angle, and radius to a CSV file
    while True:
        try:
            data = get_line_from_serial_monitor()
            if data[0] == "P":
                for cam_port in range(4):
                    # Capture image from each camera
                    capture_image(cam_port, cam_port + 1, folder_path)
            elif data[0] == "D":
                break
            else:
                x_position, angle, radius, encoder_position = data
                if x_position is not None and radius is not None and angle is not None and encoder_position is not None:
                    temp_sensor_data.append([x_position, angle, radius, encoder_position])
                else:
                    print("No valid data received for this entry")
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt: Exiting program.")
            break
        except Exception as e:
            print(f"Error: {e}")

    # sends a signal back to the serial port
    ser.write(b"g")
    print(f"Sent to serial: 'g'")

    # Close serial port
    ser.close()
    print("Serial port closed.")

    average_values(temp_sensor_data, folder_path)  # Pass folder_path here

if __name__ == "__main__":
    main()
