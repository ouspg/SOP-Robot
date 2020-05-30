# ROS-speech-separation

### Requirements

see [requirements.txt](mic_array/src/requirements.txt)

### Usage

Add mic_array package to ROS workspace and create/get state dictionary file for neural network model.

Run publisher package:
  ```shell
    rosrun mic_array publisher.py --config tune/train.yaml --state_dict tune/state_dict.pkl
  ```

For training see:
https://github.com/funcwj/uPIT-for-speech-separation

## Reference

* Kolbæk M, Yu D, Tan Z H, et al. Multitalker speech separation with utterance-level permutation invariant training of deep recurrent neural networks[J]. IEEE/ACM Transactions on Audio, Speech and Language Processing (TASLP), 2017, 25(10): 1901-1913.
