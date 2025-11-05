import torch
import numpy as np
from PIL import Image
from pathlib import Path

def run_sam2_prediction(points, image_filename):
    try:
        BASE_DIR = Path(__file__).resolve().parent
        SAM2_DIR = BASE_DIR / "SAM2"

        sam2_checkpoint = SAM2_DIR / "checkpoints" / "sam2.1_hiera_large.pt"
        model_cfg = SAM2_DIR / "configs" / "sam2.1" / "sam2.1_hiera_l.yaml"

        # Verify files exist
        if not sam2_checkpoint.exists():
            raise FileNotFoundError(f"SAM2 checkpoint not found: {sam2_checkpoint}")
        if not model_cfg.exists():
            raise FileNotFoundError(f"SAM2 config not found: {model_cfg}")

        print(f" Checkpoint path: {sam2_checkpoint}")
        print(f" Config path: {model_cfg}")

        # Load images - support both filename and full path
        image_path = Path(image_filename)
        print(f" Input image_filename: {image_filename}")
        print(f" is_absolute: {image_path.is_absolute()}")
        print(f" exists (before check): {image_path.exists()}")
        
        if image_path.is_absolute():
            # Full path provided - check if it exists or is a symlink
            if image_path.exists() or image_path.is_symlink():
                # Resolve symlink to get actual path
                image_path = image_path.resolve()
                print(f" Resolved absolute path: {image_path}")
            else:
                raise FileNotFoundError(f"Image not found at absolute path: {image_filename}")
        else:
            # Filename provided, look in extracted_data/images
            IMAGES_DIR = BASE_DIR / "extracted_data" / "images"
            image_path = IMAGES_DIR / image_filename

            if not image_path.exists():
                # Try to find with different extensions
                stem = Path(image_filename).stem
                for ext in ['.jpg', '.jpeg', '.png', '.tif', '.tiff']:
                    potential_path = IMAGES_DIR / f"{stem}{ext}"
                    if potential_path.exists():
                        image_path = potential_path
                        break

            if not image_path.exists():
                raise FileNotFoundError(f"Image not found: {image_filename}")

        original_image = np.array(Image.open(image_path))
        print(f" Loaded image: {image_path}")

        # Load SAM2 model - simpler approach that avoids Hydra issues
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        from omegaconf import OmegaConf
        from hydra.utils import instantiate

        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f" Using device: {device}")

        # Load config
        cfg = OmegaConf.load(model_cfg)
        # Add postprocessing parameters for better mask quality
        if "sam_mask_decoder_extra_args" not in cfg.model:
            cfg.model.sam_mask_decoder_extra_args = {}
        cfg.model.sam_mask_decoder_extra_args.dynamic_multimask_via_stability = True
        cfg.model.sam_mask_decoder_extra_args.dynamic_multimask_stability_delta = 0.05
        cfg.model.sam_mask_decoder_extra_args.dynamic_multimask_stability_thresh = 0.98
        OmegaConf.resolve(cfg)
        
        # Build and load model
        sam2_model = instantiate(cfg.model, _recursive_=True)
        state_dict = torch.load(sam2_checkpoint, map_location=device, weights_only=True)
        if "model" in state_dict:
            state_dict = state_dict["model"]
        sam2_model.load_state_dict(state_dict)
        sam2_model = sam2_model.to(device)
        sam2_model.eval()
        
        predictor = SAM2ImagePredictor(sam2_model)
        predictor.set_image(original_image)

        # Run prediction
        input_label = np.ones(len(points), dtype=np.int32)
        masks, scores, _ = predictor.predict(
            point_coords=points,
            point_labels=input_label,
            multimask_output=False,
        )

        # Create overlay
        best_mask = masks[0]  # Take the first (and best) mask

        # Create overlay
        overlay = np.zeros((*best_mask.shape, 4), dtype=np.float32)
        overlay[best_mask > 0] = [30 / 255, 144 / 255, 255 / 255, 0.5]

        # Composite overlay with original image
        base = Image.fromarray(original_image).convert("RGBA")
        overlay_img = Image.fromarray((overlay * 255).astype(np.uint8))
        final_img = Image.alpha_composite(base, overlay_img)

        # Save outputs
        output_dir = BASE_DIR / "outputs"
        output_dir.mkdir(exist_ok=True)

        # Generate filenames based on input image
        stem = Path(image_filename).stem
        overlay_path = output_dir / f"{stem}_overlay.png"
        mask_path = output_dir / f"{stem}_binary_mask.png"

        # Save images
        final_img.save(overlay_path)
        Image.fromarray((best_mask * 255).astype(np.uint8)).save(mask_path)

        print(f" Saved overlay: {overlay_path}")
        print(f" Saved binary mask: {mask_path}")

        return {
            "success": True,
            "overlay_path": str(overlay_path),
            "mask_path": str(mask_path),
            "mask_shape": best_mask.shape,
            "score": float(scores[0]),
            "image_size": original_image.shape
        }

    except Exception as e:
        print(f" SAM2 prediction failed: {e}")
        raise e