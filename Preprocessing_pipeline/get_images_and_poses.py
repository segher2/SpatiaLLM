import numpy as np
import cv2
import pye57
import argparse
import os
import json
import logging

# Setup basic logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


def extract_and_save_images_and_metadata(e57_path, output_folder):
    logging.info("Loading E57 file...")
    e57 = pye57.E57(e57_path)
    imf = e57.image_file
    root = imf.root()

    logging.info("File loaded successfully.")

    if not root['images2D']:
        logging.warning("File contains no 2D images. Exiting...")
        return

    images_output_folder = os.path.join(output_folder, "images")
    metadata_output_folder = os.path.join(output_folder, "metadata")
    os.makedirs(images_output_folder, exist_ok=True)
    os.makedirs(metadata_output_folder, exist_ok=True)


    for image_idx, image2D in enumerate(root['images2D']):
        logging.info(f"Processing image {image_idx}...")

        pinhole = image2D['pinholeRepresentation'] if 'pinholeRepresentation' in image2D else image2D[
            'sphericalRepresentation']
        jpeg_image = pinhole['jpegImage']
        jpeg_image_data = np.zeros(shape=jpeg_image.byteCount(), dtype=np.uint8)
        jpeg_image.read(jpeg_image_data, 0, jpeg_image.byteCount())
        image = cv2.imdecode(jpeg_image_data, cv2.IMREAD_COLOR)

        image_name = str(image2D['name'].value())
        image_path = os.path.join(images_output_folder, f"{image_name}")
        cv2.imwrite(image_path, image)
        logging.info(f"Saved image to {image_path}")

        print(image2D['pose'])

        try:
            translation = image2D['pose']['translation']
            rotation = image2D['pose']['rotation']
            x = float(translation['x'].value())
            y = float(translation['y'].value())
            z = float(translation['z'].value())
            rx = float(rotation['x'].value())
            ry = float(rotation['y'].value())
            rz = float(rotation['z'].value())
            rw = float(rotation['w'].value())
            logging.info(f"Coords: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}, rw={rw}")

            # Save pose information in metadata
            metadata = {
                'name': image_name,
                'translation': {'x': x, 'y': y, 'z': z},
                'rotation': {'x': rx, 'y': ry, 'z': rz, 'w': rw}
            }
        except Exception:
            metadata = {'name': image_name}
        # slice to leave .jpg out of image_name
        metadata_path = os.path.join(metadata_output_folder, f"{image_name[:-4]}.json")
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=4)
        logging.info(f"Saved metadata to {metadata_path}")

    logging.info("All images and metadata processed successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract and save 2D images and metadata from an E57 file.')
    parser.add_argument('e57', help='Path to the E57 file')
    parser.add_argument('--outfolder', default='extracted_data',
                        help='Output folder for the extracted images and metadata')
    args = parser.parse_args()

    extract_and_save_images_and_metadata(args.e57, args.outfolder)


