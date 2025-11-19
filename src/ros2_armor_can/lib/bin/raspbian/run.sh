export LD_LIBRARY_PATH=.:$(LD_LIBRARY_PATH)
while true
do
  sleep 0.2s && ./bmapi_test 0 tx 1000 10 1 &
  ./bmapi_test 1 rx 10000
done
