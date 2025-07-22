# DOC-Depth: A novel approach for dense depth ground truth generation


[![Paper](https://img.shields.io/badge/arXiv-2502.02144-brightgreen)](https://arxiv.org/abs/2502.02144)
[![Conference](https://img.shields.io/badge/IEEE_IV-2025-blue)](https://ieee-iv.org/2025/)
[![Project Page](https://img.shields.io/badge/Project-page-red)](https://simondemoreau.github.io/DOC-Depth)

![Paper concept](assets/paper_concept.png) 
Official implementation of the DOC-Depth method. 

If you use our method in your research, please cite :
```bibtex
@inproceedings{deMoreau2024doc,
  title = {DOC-Depth: A novel approach for dense depth ground truth generation},
  author = {De Moreau, Simon and Corsia, Mathias and Bouchiba, Hassan and Almehio, Yasser and Bursuc, Andrei and El-Idrissi, Hafid and Moutarde, Fabien},
  booktitle = {2025 IEEE Intelligent Vehicles Symposium (IV)},
  year = {2025},
}
```

## Dense Depth KITTI annotations
Please visit our [project page](https://simondemoreau.github.io/DOC-Depth) to download the dense annotations of KITTI. 

## Calibration
The first step of the pipeline is to calibrate together LiDAR and Camera. See the [Calibration](Calibration) folder to use our tool.

## Recording
The easiest way to record your dataset is to use [ROS](https://ros.org/) to record all your sensors into ".bag" files.

## Preprocessing
After recording, you must use our pre-processing pipeline with SLAM and DOC to obtain a dense and classified reconstruction of your record. See the [Preprocessing](Preprocessing) folder for more informations. 

## Rendering
Finally, you can use our tool to apply our composite rendering to the classified LiDAR frames and obtain your dense depth. See the [Rendering](Rendering) folder to access our tool. 

## License 
This repository is licensed under the [Apache License 2.0](LICENSE).