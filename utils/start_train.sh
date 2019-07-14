# From models directory
gcloud ml-engine jobs submit training `whoami`_object_detection_`date +%s` \
    --job-dir=gs://capstone_sim/train \
    --packages dist/object_detection-0.1.tar.gz,slim/dist/slim-0.1.tar.gz \
    --module-name object_detection.model_main \
    --region asia-southeast1 \
    --config google_cloud_config.yml \
    -- \
    --train_dir=gs://capstone_sim/train \
    --pipeline_config_path=gs://capstone_sim/data/config