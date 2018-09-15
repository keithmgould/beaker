import matplotlib.pyplot as plt
import numpy as np
import math

# accepts the history array from Keras
class LossCharter:

  def chart(self, history, name):
    val_loss = plt.plot(history["val_loss"],'r--', label='val loss')
    loss = plt.plot(history["loss"], 'b--', label='loss')
    plt.title(name)
    plt.legend()
    plt.savefig("{}.png".format(name))