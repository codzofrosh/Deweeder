!pip install ultralytics -q
!pip install tflite-support -q

import os
import yaml
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import torch
from ultralytics import YOLO
from google.colab import drive, files
from IPython.display import display, Image as IPImage
import warnings
warnings.filterwarnings('ignore')
import json
from datetime import datetime # Import datetime
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f" Using device: {device}")
if device == 'cuda':
    print(f"   GPU: {torch.cuda.get_device_name(0)}")
    print(f"   Memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
else:
    print("     No GPU found. Training will be slower on CPU.")
    print("     Tip: Go to Runtime → Change runtime type → GPU")

# Set your dataset path here
DATASET_PATH = "/content/drive/MyDrive/WeedDetection"  # Update this to your dataset location
DRIVE_PROJECT_PATH = "/content/drive/MyDrive/WeedDetection/weed_detector_project"
CHECKPOINT_DIR = f"{DRIVE_PROJECT_PATH}/checkpoints"
RESULTS_DIR = f"{DRIVE_PROJECT_PATH}/results"
os.makedirs(CHECKPOINT_DIR, exist_ok=True)
os.makedirs(RESULTS_DIR, exist_ok=True)
def setup_dataset():
    """Sets up dataset by creating data.yaml and verifying structure."""

    # Create data.yaml for YOLO
    data_yaml = {
        'path': DATASET_PATH,
        'train': 'train/images',  # or just 'train' depending on your structure
        'val': 'valid/images',      # or just 'val'
        'test': 'test/images',    # or just 'test'
        'nc': 2,  # number of classes
        'names': ['crop', 'weed']  # class names
    }

    # Save data.yaml
    yaml_path = os.path.join(DATASET_PATH, 'data.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(data_yaml, f)

    print(f"✓ data.yaml created at: {yaml_path}")

    # Verify dataset structure
    print("\n📊 Dataset Structure:")
    for split in ['train', 'valid', 'test']:
        img_path = os.path.join(DATASET_PATH, split, 'images')
        lbl_path = os.path.join(DATASET_PATH, split, 'labels')

        if os.path.exists(img_path):
            n_images = len([f for f in os.listdir(img_path) if f.endswith(('.jpg', '.jpeg', '.png'))])
            print(f"{split}: {n_images} images")
        else:
            # Try without 'images' subfolder
            alt_path = os.path.join(DATASET_PATH, split)
            if os.path.exists(alt_path):
                n_images = len([f for f in os.listdir(alt_path) if f.endswith(('.jpg', '.jpeg', '.png'))])
                print(f"{split}: {n_images} images")

    return yaml_path
TRAINING_CONFIG = {
    'experiments': [
        {
            'name': 'yolov8n_fast',
            'model': 'yolov8n.pt',
            'epochs': 100,
            'batch': 32 if device == 'cuda' else 8,  # Smaller batch for CPU
            'patience': 20,
            'description': 'Fastest model for RPi'
        },
        {
            'name': 'yolov8n_balanced',
            'model': 'yolov8n.pt',
            'epochs': 150,
            'batch': 24 if device == 'cuda' else 8,
            'patience': 25,
            'description': 'Balanced speed/accuracy'
        },
        {
            'name': 'yolov8s_accurate',
            'model': 'yolov8s.pt',
            'epochs': 100,
            'batch': 16 if device == 'cuda' else 4,
            'patience': 20,
            'description': 'Best accuracy'
        }
    ]
}
# 4. CHECKPOINT MANAGEMENT
# =====================================
class CheckpointManager:
    def __init__(self, checkpoint_dir):
        self.checkpoint_dir = checkpoint_dir
        self.state_file = os.path.join(checkpoint_dir, 'training_state.json')

    def save_state(self, experiment_name, state_info):
        """Save training state"""
        states = self.load_all_states()
        states[experiment_name] = {
            **state_info,
            'last_updated': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }

        with open(self.state_file, 'w') as f:
            json.dump(states, f, indent=2)

    def load_all_states(self):
        """Load all training states"""
        if os.path.exists(self.state_file):
            with open(self.state_file, 'r') as f:
                return json.load(f)
        return {}

    def get_state(self, experiment_name):
        """Get state for specific experiment"""
        states = self.load_all_states()
        return states.get(experiment_name, {})

    def get_checkpoint_path(self, experiment_name):
        """Get checkpoint path for experiment"""
        return os.path.join(self.checkpoint_dir, experiment_name, 'weights', 'last.pt')

    def can_resume(self, experiment_name):
        """Check if experiment can be resumed"""
        checkpoint_path = self.get_checkpoint_path(experiment_name)
        return os.path.exists(checkpoint_path)

# Initialize checkpoint manager
checkpoint_manager = CheckpointManager(CHECKPOINT_DIR)
# 5. TRAINING FUNCTION WITH RESUME
# =====================================
def train_experiment(exp_config, force_restart=False):
    """Train single experiment with resume capability"""

    exp_name = exp_config['name']
    print(f"\n{'='*60}")
    print(f"🚀 Experiment: {exp_name}")
    print(f"   Description: {exp_config['description']}")
    print(f"{'='*60}")

    # Check previous state
    state = checkpoint_manager.get_state(exp_name)
    checkpoint_path = checkpoint_manager.get_checkpoint_path(exp_name)

    # Determine if we should resume
    resume = False
    start_epoch = 0

    if checkpoint_manager.can_resume(exp_name) and not force_restart:
        print(f"✅ Found checkpoint at: {checkpoint_path}")

        # Check epochs completed
        if 'epochs_completed' in state:
            start_epoch = state['epochs_completed']
            print(f"   Completed epochs: {start_epoch}/{exp_config['epochs']}")

            if start_epoch >= exp_config['epochs']:
                print("   ✅ Training already completed!")
                return load_completed_model(exp_name)

        resume = True
        model = YOLO(checkpoint_path)
        print("   Resuming training...")
    else:
        print("   Starting new training...")
        model = YOLO(exp_config['model'])

    # Adjust parameters for CPU if needed
    batch_size = exp_config['batch']
    workers = 8

    if device == 'cpu':
        print("   ⚠️  CPU mode: Using reduced batch size and workers")
        batch_size = min(batch_size, 8)  # Max batch size 8 for CPU
        workers = 2  # Fewer workers for CPU

    # Training parameters
    train_params = {
        'data': os.path.join(DATASET_PATH, 'data.yaml'),
        'epochs': exp_config['epochs'],
        'batch': batch_size,
        'imgsz': 640,
        'device': device,
        'workers': workers,
        'project': CHECKPOINT_DIR,
        'name': exp_name,
        'exist_ok': True,
        'resume': resume,
        'patience': exp_config['patience'],
        'save': True,
        'save_period': 5,  # Save every 5 epochs
        'cache': device == 'cuda',  # Only cache on GPU
        'amp': device == 'cuda',  # Mixed precision only on GPU

        # Optimizer settings
        'optimizer': 'AdamW',
        'lr0': 0.002,
        'lrf': 0.01,
        'momentum': 0.937,
        'weight_decay': 0.0005,

        # Augmentation
        'degrees': 10,
        'translate': 0.1,
        'scale': 0.5,
        'flipud': 0.2,
        'fliplr': 0.5,
        'mosaic': 0.8 if device == 'cuda' else 0.3,  # Less augmentation on CPU
        'mixup': 0.1 if device == 'cuda' else 0.0,
    }

    try:
        # Train model
        print(f"\n🏃 Training on {device.upper()}...")
        results = model.train(**train_params)

        # Save state
        checkpoint_manager.save_state(exp_name, {
            'status': 'completed',
            'epochs_completed': exp_config['epochs'],
            'best_metrics': {
                'mAP50': float(results.results_dict.get('metrics/mAP50(B)', 0)),
                'mAP50-95': float(results.results_dict.get('metrics/mAP50-95(B)', 0))
            }
        })

        # Validate on test set
        print("\n📊 Validating on test set...")
        test_metrics = model.val(split='test')

        # Export models
        print("\n📦 Exporting models...")
        export_models(model, exp_name)

        return {
            'model': model,
            'train_results': results,
            'test_metrics': test_metrics,
            'status': 'completed'
        }

    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user")
        # Save current state
        epochs_done = len(pd.read_csv(f"{CHECKPOINT_DIR}/{exp_name}/results.csv"))
        checkpoint_manager.save_state(exp_name, {
            'status': 'interrupted',
            'epochs_completed': epochs_done
        })
        raise

    except Exception as e:
        print(f"\n❌ Error during training: {e}")
        checkpoint_manager.save_state(exp_name, {
            'status': 'failed',
            'error': str(e)
        })
        return None
# 7. LOAD COMPLETED MODEL
# =====================================
def load_completed_model(exp_name):
    """Load a previously completed model"""

    best_path = os.path.join(CHECKPOINT_DIR, exp_name, 'weights', 'best.pt')

    if os.path.exists(best_path):
        model = YOLO(best_path)
        test_metrics = model.val(split='test')

        return {
            'model': model,
            'train_results': None,
            'test_metrics': test_metrics,
            'status': 'loaded'
        }

    return None
# 8. TRAINING STATUS DASHBOARD
# =====================================
def show_training_status():
    """Display current training status"""

    print("\n📊 TRAINING STATUS DASHBOARD")
    print("="*70)

    states = checkpoint_manager.load_all_states()
    status_data = []

    for exp in TRAINING_CONFIG['experiments']:
        exp_name = exp['name']
        state = states.get(exp_name, {})

        # Check actual checkpoint
        checkpoint_exists = checkpoint_manager.can_resume(exp_name)

        # Get current epoch from results.csv if exists
        current_epoch = state.get('epochs_completed', 0)
        results_path = os.path.join(CHECKPOINT_DIR, exp_name, 'results.csv')
        if os.path.exists(results_path):
            try:
                df = pd.read_csv(results_path)
                current_epoch = len(df)
            except:
                pass

        status_data.append({
            'Experiment': exp_name,
            'Target Epochs': exp['epochs'],
            'Current Epoch': current_epoch,
            'Status': state.get('status', 'not started'),
            'Can Resume': '✅' if checkpoint_exists else '❌',
            'Last Updated': state.get('last_updated', 'N/A')
        })

    df_status = pd.DataFrame(status_data)
    display(df_status)

    return df_status
# 9. MAIN TRAINING ORCHESTRATOR
# =====================================
def train_all_experiments(force_restart=False, skip_completed=True):
    """Train all experiments with smart resume"""

    print("🤖 YOLO WEED DETECTION TRAINING")
    print(f"   Device: {device}")
    print(f"   Dataset: {DATASET_PATH}")
    print(f"   Checkpoints: {CHECKPOINT_DIR}")
    print("="*70)

    # Setup dataset
    yaml_path = setup_dataset()

    # Show current status
    show_training_status()

    # Train each experiment
    all_results = []

    for exp_config in TRAINING_CONFIG['experiments']:
        exp_name = exp_config['name']

        # Check if should skip
        if skip_completed and not force_restart:
            state = checkpoint_manager.get_state(exp_name)
            if state.get('status') == 'completed':
                print(f"\n✅ {exp_name} already completed. Loading results...")
                result = load_completed_model(exp_name)
                if result:
                    all_results.append((exp_name, result))
                continue

        # Train
        try:
            result = train_experiment(exp_config, force_restart)
            if result:
                all_results.append((exp_name, result))
        except KeyboardInterrupt:
            print("\n⚠️  Training interrupted. Progress saved.")
            print("   Run train_all_experiments() again to resume.")
            break
        except Exception as e:
            print(f"\n❌ Failed to train {exp_name}: {e}")
            continue

    # Show final results
    if all_results:
        show_final_results(all_results)

    return all_results
#10. RESULTS VISUALIZATION
# =====================================
def show_final_results(all_results):
    """Display final results comparison"""

    print("\n" + "="*70)
    print("📊 FINAL RESULTS COMPARISON")
    print("="*70)

    results_data = []

    for exp_name, result in all_results:
        if result and result['test_metrics']:
            metrics = result['test_metrics']

            # Average metrics
            results_data.append({
                'Experiment': exp_name,
                'Status': result['status'],
                'mAP50': np.mean(metrics.box.map50),
                'mAP50-95': np.mean(metrics.box.map),
                'Precision': np.mean(metrics.box.p),
                'Recall': np.mean(metrics.box.r),
                'Speed (ms)': metrics.speed['inference']
            })

    if results_data:
        df_results = pd.DataFrame(results_data)
        df_results = df_results.round(3)
        display(df_results)

        # Save results
        df_results.to_csv(f"{RESULTS_DIR}/final_comparison.csv", index=False)

        # Plot comparison
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))

        metrics_to_plot = ['mAP50', 'Precision', 'Recall']
        for idx, metric in enumerate(metrics_to_plot):
            df_results.plot(x='Experiment', y=metric, kind='bar', ax=axes[idx])
            axes[idx].set_title(f'{metric} Comparison')
            axes[idx].set_ylim(0, 1)
            axes[idx].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(f"{RESULTS_DIR}/comparison_plot.png", dpi=150)
        plt.show()
# 11. QUICK RESUME FUNCTION
# =====================================
def quick_resume():
    """Quick function to resume after disconnect"""

    print("🔄 QUICK RESUME MODE")
    print("="*50)

    # Check device
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Device: {device}")

    # Show status
    show_training_status()

    print("\n📌 Resuming incomplete experiments...")
    train_all_experiments(force_restart=False, skip_completed=True)

# 12. DEPLOYMENT PACKAGE CREATOR
# =====================================
def create_deployment_package(exp_name='yolov8n_balanced'):
    """Create deployment package for Raspberry Pi"""

    print(f"\n📦 Creating deployment package for {exp_name}...")

    # Paths
    model_path = os.path.join(CHECKPOINT_DIR, exp_name, 'weights', 'best_int8.tflite')

    if not os.path.exists(model_path):
        print("⚠️  TFLite model not found. Exporting...")
        model = YOLO(os.path.join(CHECKPOINT_DIR, exp_name, 'weights', 'best.pt'))
        model.export(format='tflite', int8=True)

    # Create inference script
    inference_code = '''#!/usr/bin/env python3
"""
Weed Detection for Raspberry Pi
Optimized for real-time inference
"""

import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
import time

class WeedDetector:
    def __init__(self, model_path='best_int8.tflite'):
        self.interpreter = Interpreter(model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.input_shape = self.input_details[0]['shape']
        self.classes = ['crop', 'weed']

    def detect(self, image):
        # Preprocess
        input_size = (self.input_shape[1], self.input_shape[2])
        img = cv2.resize(image, input_size)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        # Inference
        start = time.time()
        self.interpreter.set_tensor(self.input_details[0]['index'], img)
        self.interpreter.invoke()
        inference_time = (time.time() - start) * 1000

        # Get outputs
        output = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        return output, inference_time

# Usage
detector = WeedDetector()
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    detections, inf_time = detector.detect(frame)

    # Process detections and draw
    # ... (detection processing code)

    cv2.putText(frame, f'FPS: {1000/inf_time:.1f}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Weed Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
'''

    # Save deployment files
    deploy_dir = f"{RESULTS_DIR}/raspberry_pi_deployment"
    os.makedirs(deploy_dir, exist_ok=True)

    with open(f"{deploy_dir}/weed_detector.py", 'w') as f:
        f.write(inference_code)

    print(f"✅ Deployment package ready at: {deploy_dir}")

# =====================================
# EXECUTION
# =====================================

# For first time run:
print("📌 To start training, run:")
print("   train_all_experiments()")
print("\n📌 After disconnect, run:")
print("   quick_resume()")

# Uncomment to start training immediately:
train_all_experiments()
#quick_resume()
