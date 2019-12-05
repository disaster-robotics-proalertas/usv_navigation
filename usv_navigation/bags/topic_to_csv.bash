var="exp2"
rostopic echo -b ${var}.bag -p /sailboat/state > data_tst.txt
python read_csv_bag.py
rm data_tst.txt
mv data_out.csv ${var}_cartesian.csv
mv ${var}* experiments