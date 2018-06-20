# import the inspect_checkpoint library
import tensorflow as tf
from tensorflow.python.tools import inspect_checkpoint as chkp

# print all tensors in checkpoint file
chkp.print_tensors_in_checkpoint_file(tf.train.latest_checkpoint('./models/'), tensor_name='', all_tensors=True)

# # tensor_name:  v1
# # [ 1.  1.  1.]
# # tensor_name:  v2
# # [-1. -1. -1. -1. -1.]

# # print only tensor v1 in checkpoint file
# chkp.print_tensors_in_checkpoint_file("/tmp/model.ckpt", tensor_name='v1', all_tensors=False)

# # tensor_name:  v1
# # [ 1.  1.  1.]

# # print only tensor v2 in checkpoint file
# chkp.print_tensors_in_checkpoint_file("/tmp/model.ckpt", tensor_name='v2', all_tensors=False)

# # tensor_name:  v2
# # [-1. -1. -1. -1. -1.]