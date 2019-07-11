import rospy

class TLClassifierParams(object):
    __namespace__ = lambda param, default_value : rospy.get_param("/tl_classifier/{}".format(param), default_value)

    ####################################################################
    # All parameters are of teh format: "/tl_classifier/<param-name>"
    ####################################################################
    MODEL_FILE = __namespace__('model_file', "")
    COLLECT_DATA = __namespace__('collect_data', False)
    DATA_FOLDER = __namespace__('data_folder', "../../../images")

