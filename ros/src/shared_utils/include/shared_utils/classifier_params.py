import rospy

class TLClassifierParams(object):
    __namespace__ = lambda param, default_value : rospy.get_param("/tl_classifier/{}".format(param), default_value)

    ####################################################################
    # All parameters are of teh format: "/tl_classifier/<param-name>"
    ####################################################################
    MODEL_FILE = __namespace__('model_file', "")
    COLLECT_DATA = __namespace__('collect_data', False)
    DATA_FOLDER = __namespace__('data_folder', "../../../images")

    SAVING_IMAGES = __namespace__('save_images', False) # 0 = False / 1 = True
    TEST_MODE = __namespace__('test_mode', False) # 0 = False / 1 = True

    # sending only every IMAGE_DEBOUNCE-th image from simulator to the topic; doesn't work, when saving images
    IMAGE_DEBOUNCE = __namespace__('image_debounce', 3)
