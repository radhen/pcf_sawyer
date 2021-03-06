#!/usr/bin/env python3
import sys
import h5py
import numpy as np
from keras import backend as K
from keras.models import load_model
from keras.models import model_from_json
#np.set_printoptions(threshold=np.nan)

f= h5py.File('nn_motion3_reg_weights.best.hdf5', 'r')
# model= load_model('weights.best.hdf5')
json_file = open('model.nn_motion3_reg.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
model = model_from_json(loaded_model_json)
textFile= open("wr_layer_{}.txt".format(sys.argv[1]), "w")


layer= int(sys.argv[1])
print(model.summary())
input_size= model.layers[layer].input_shape
weights= model.layers[layer].get_weights()[0] #weights
biases= model.layers[layer].get_weights()[1] #biases
print("input_shape:")
print(input_size)
print("Shape Weights:")
print(weights.shape)
print("Shape biases:")
print(biases.shape)
print("layer config:")
print(model.layers[layer].get_config())

strWeights= str(list(weights))
strBiases= str(list(biases))

strWeights= strWeights.replace('[', '{')
strWeights= strWeights.replace(']', '}')
strWeights= strWeights.replace('dtype=float32),', '')
strWeights= strWeights.replace('array(', '')
strWeights= strWeights.replace(', dtype=float32)', '')
print()

textFile.write(strWeights)
strBiases= strBiases.replace('[', '{')
strBiases= strBiases.replace(']', '}')
#strBiases= strBiases.replace(' ', ',')

print()
print(strBiases)

# print(model.layers[0])
# print("Keys: %s" % f.keys())
# a_group_key= list(f.keys())[0]
#
# test= np.ones((1, 50, 2))
# layer_outs=functor([test, layer])
# print(layer_outs)
#
#
# data= list(f[a_group_key])
# print(data)

