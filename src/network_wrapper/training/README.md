# Training
This is a small example to describe what the process looks like for training a new neural network object detector for AUV. 

### Install Dependencies
To start training, you need to make sure your evironment and versions are correct. My first suggestion, is to try the install.sh script to install all the major dependencies. It will also download the `tf_trt_models/third_party/models` repo, which is needed for training and freezing graphs. If that fails or you're getting weird errors, try tensorflow's official [documentation](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md). If you want, you may also try to use an [annacond environemnt](https://www.anaconda.com/tensorflow-in-anaconda/). They try to make standardized environments where all your dependencies are taken care of.

### Make jpeg from MP4
The following command will split your video up into jpegs. `-i` is the path to the video file. `-qscale:v` followed by a number from 2-31 this indicates quality. 2 being the best, 31 being the worst. `-r` is the number of images per second to output. Think of this as how many pictures you want. EX: if you have a 10 second video, a value of `-r 2.5` will output 10*2.5 = 25 pictures. The last argument is where the images should output to, and what the format is that they should look like. `%04d` means 4 decimal numbers where they are all zero.
```bash
ffmpeg -i input_video.MP4 -qscale:v 2 -r 2.5 output_directory/output_image_%04d.jpeg
```
This would output videos in the `output_directory` with the names `output_image_0001.jpeg`, `output_image_0002.jpeg`, and so on.

### Install Label Tool
You can, in theory, use any labeling tool that outputs to PASCAL VOC format to train with, but I like to use this one called [LabelImg](https://github.com/tzutalin/labelImg). To download, do the following:
```bash
git clone https://github.com/tzutalin/labelImg.git
cd labelImg
pip3 install pyqt5 lxml # Install qt and lxml by pip
make qt5py3
```
To run the program, just navigate to where you cloned the repo and use `python3 labelImg.py`

When actually labeling the data, I would recomend making a new directory in the `training` directory with the same structure as the `example_training_layout`. So, just three subdirectories, `annotations`, `images`, and `output_files`.  This will make it easier to keep track of when we convert to TFRecords.

When you are using the labeling tool, I would reccomend using the Change Save Dir to change directory where the annotations get saved to from the default to your new `annotations` directory. Also, turn on auto save mode, so you don't have to tap save every time. 

For a quick reference, `w` puts you in create box mode. `d` goes to the next image.

### Create TFRecords
To Train using tensorflow, we need our tagged data to be in a format tensorflow understands. This is called TFRecords and they have a `.record` extension. At the end of this, we should end up with two .record files. One for training data, and one for evaluation purposes. 

So, to do this, we are going to do four things:
  1. Combine our `.xml` files in the `annotations` directory into one `.csv` file
  2. Create a `label_map.pbtxt` of our tagged data to hand to tensorflow
  3. Split our single `.csv` file into two. One for training and one for evaluation.
  4. Convert the two new training and evaluation `.csv` files into TFRecord format with `.record` extension.

To do this, cd into the `scripts` directory. Each of the scrips here can be run as the following:
```bash
python3 <script-name> --help
```
This will show you how the script is supposed to be run, and what arguments are required to do the steps mentioned above.

For your convience, I will show you an example for how the outputs of the `example_training_layout` were created:
```bash
python3 generate_csv.py xml ../example_training_layout/annotations ../example_training_layout/output_files/test_csv.csv
python3 generate_pbtxt.py csv ../example_training_layout/output_files/test_csv.csv ../example_training_layout/output_files/label_map.pbtxt
python3 generate_train_eval.py ../example_training_layout/output_files/test_csv.csv 
python3 generate_tfrecord.py ../example_training_layout/output_files/test_csv_train.csv ../example_training_layout/output_files/label_map.pbtxt ../example_training_layout/images/ ../example_training_layout/output_files/example_train.record
python3 generate_tfrecord.py ../example_training_layout/output_files/test_csv_eval.csv ../example_training_layout/output_files/label_map.pbtxt ../example_training_layout/images/ ../example_training_layout/output_files/example_eval.record
```

### Pick Network Model


### Start Training
Now that we have everythong setup and ready, it's time to actually
