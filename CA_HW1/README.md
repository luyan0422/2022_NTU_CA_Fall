## Compurter Architecture 2022 Fall homework 1

### Homework 1-1 <br>
The code is refer to professor's slides <br>

### Homewoek 1-2 <br>
I wrote this Median Filter function in C first, and then converted it into RISC-V. <br>
Wride down my code below for reference <br><br>

```
//filter funtion
void median_filter(int **target, int **result, int row, int col){
    int new_row = row - 2, new_col = col - 2;
    for(int i = 1; i < row - 1; i++){ // 邊界不用處理
        for(int j = 1; j < col - 1; j++){ // 邊界不用處理
            int count = 0;
            int filter_window[9];
            //get the value
            for (int tmp_i = i - 1; tmp_i < i + 2; tmp_i++){ 
                for (int tmp_j = j - 1; tmp_j < j + 2; tmp_j ++) {
                    filter_window[count] = *((int *)target + tmp_i * row + tmp_j);
                    count++;
                }
            }

            //get the 5th value
            for (int m = 0; m < 5; m++){  // once get the median value, stop sorting
                int min = m;  
                for (int n = m + 1; n < 9; n++){  
                    if (filter_window[n] < filter_window[min])  
                           min = n;  
                }
                int temp = filter_window[m];  
                filter_window[m] = filter_window[min];  
                filter_window[min] = temp;  
            }
            *((int *)result + (i - 1) * new_row + (j - 1)) = filter_window[4]; 
        }
    }
    return ;
}
```





