Inlcude libraries in current terminal session:
export LD_LIBRARY_PATH=third_party_libraries/or-tools/lib:$LD_LIBRARY_PATH

Create executable:
g++ -std=c++17 my_test_scripts/ILP_test.cpp -o my_test_scripts/ILP_test -Ithird_party_libraries/or-tools/include -Lthird_party_libraries/or-tools/lib -lortools

Run executable:
./my_test_scripts/ILP_test