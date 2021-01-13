#include <iostream>
#include <string>
#include <cctype>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <initializer_list>

using namespace std;
using std::cout;

float MLX90641To[192];

float cal_mean(float col[12])
{
    float mean = 0;
    for (int i = 0; i <= 11; i++)
    {
        
    }
}

float cal_var(float col[12])
{
    float col_mean
}

class Frame
{
private:
    float col_0[12];
    float col_1[12];
    float col_2[12];
    float col_3[12];
    float col_4[12];
    float col_5[12];
    float col_6[12];
    float col_7[12];
    float col_8[12];
    float col_9[12];
    float col_10[12];
    float col_11[12];
    float col_12[12];
    float col_13[12];
    float col_14[12];
    float col_15[12];

    float col_var[16];

public:
    //初始化col_0-15
    Frame(float a[192])
    {
        for (int j = 0; j <= 11; j++)
        {
            col_0[j] = a[j * 16];
            col_1[j] = a[j * 16 + 1];
            col_2[j] = a[j * 16 + 2];
            col_3[j] = a[j * 16 + 3];
            col_4[j] = a[j * 16 + 4];
            col_5[j] = a[j * 16 + 5];
            col_6[j] = a[j * 16 + 6];
            col_7[j] = a[j * 16 + 7];
            col_8[j] = a[j * 16 + 8];
            col_9[j] = a[j * 16 + +9];
            col_10[j] = a[j * 16 + 10];
            col_11[j] = a[j * 16 + 11];
            col_12[j] = a[j * 16 + 12];
            col_13[j] = a[j * 16 + 13];
            col_14[j] = a[j * 16 + 14];
            col_15[j] = a[j * 16 + 15];
        }
        col_var[0] =
    }
};

int main()
{
    float a[] = {1, 1, 1, 1, 1, 1, 1, 1};
}
