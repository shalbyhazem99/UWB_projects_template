# Signal Analysis Tools

This folder contains three Jupyter Notebook–based tools designed to support the analysis, visualization, and inspection of logs.

## Tools

- **[extract_breathing_rate_crop_windows.ipynb](extract_breathing_rate_crop_windows.ipynb)**  
  Extracts the breathing rate from time-series signals, with support for cropping and analyzing specific time windows of interest.

- **[inhalation_exhalation_visualizer.ipynb](inhalation_exhalation_visualizer.ipynb)**  
  Visualizes inhalation and exhalation phases of the respiratory signal, enabling qualitative inspection of breathing patterns and segmentation accuracy.

- **[log_viewer.ipynb](log_viewer.ipynb)**
  Interactive viewer for loading and inspecting log files (e.g. TWR distance or signal logs). It can be used to visualize distances of interest and to identify which bins should be removed in order to achieve correct data alignment. This is particularly useful during preprocessing to ensure temporal and spatial consistency of the signals before analysis.

## Notes

These notebooks are intended as analysis and debugging tools and may require adaptation depending on the data format and acquisition setup.

