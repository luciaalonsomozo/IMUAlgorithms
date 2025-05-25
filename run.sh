# python3 readings_to_csv/reading_data_dmp.py
# python3 readings_to_csv/reading_data.py
python3 filters/euler_direct.py
python3 filters/complementary_filter.py
python3 filters/kalman_filter.py
gcc filters/Fusion-main/Examples/Simple/main.c filters/Fusion-main/Fusion/*.c -I Fusion -o filters/Fusion-main/Examples/Simple/main
./filters/Fusion-main/Examples/Simple/main
gcc filters/Fusion-main/Examples/Advanced/main.c filters/Fusion-main/Fusion/*.c -I Fusion -o filters/Fusion-main/Examples/Advanced/advanced
./filters/Fusion-main/Examples/Advanced/advanced
gcc filters/Fusion-main/Examples/AdvancedCalibration/main.c filters/Fusion-main/Fusion/*.c -I Fusion -o filters/Fusion-main/Examples/AdvancedCalibration/advanced
./filters/Fusion-main/Examples/AdvancedCalibration/advanced
python3 util/comparison.py