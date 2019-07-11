

#####################################

MODEL_FILE_PARAM = ("/tl_classifier/model_file", "model_file.pb")
COLLECT_DATA_PARAM = ("/tl_classifier/collect_data", False)
DATA_FOLDER_PARAM = ("/tl_classifier/data_folder", "../../../images")

SIM_MODE_PARAM = ("/sim_mode", False)

SAVING_IMAGES = False
TEST_MODE = True
IMAGE_DEBOUNCE = 3  # sending only every IMAGE_DEBOUNCE-th image from simulator to the topic; doesn't work, when saving images
