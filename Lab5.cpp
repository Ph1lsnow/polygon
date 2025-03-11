#include <iostream>
#include <vector>
#include <algorithm> // Для std::reverse и std::max_element
#include <cstdlib>   // Для rand и srand
#include <ctime>     // Для времени


using namespace std;


// Odd-Even Sort
void oddEvenSort(vector<int>& arr) {
    bool isSorted = false;
    int n = arr.size();


    while (!isSorted) {
        isSorted = true;


        // Чётный проход
        for (int i = 0; i < n - 1; i += 2) {
            if (arr[i] > arr[i + 1]) {
                swap(arr[i], arr[i + 1]);
                isSorted = false;
            }
        }


        // Нечётный проход
        for (int i = 1; i < n - 1; i += 2) {
            if (arr[i] > arr[i + 1]) {
                swap(arr[i], arr[i + 1]);
                isSorted = false;
            }
        }
    }
}


// Pancake Sort
void flip(vector<int>& arr, int k) {
    reverse(arr.begin(), arr.begin() + k);
}


void pancakeSort(vector<int>& arr) {
    int n = arr.size();


    for (int size = n; size > 1; --size) {
        // Найти индекс максимального элемента в текущем подмассиве
        int maxIdx = max_element(arr.begin(), arr.begin() + size) - arr.begin();


        // Перевернуть так, чтобы максимальный элемент оказался в начале
        if (maxIdx != size - 1) {
            if (maxIdx > 0) {
                flip(arr, maxIdx + 1);
            }
            // Перевернуть так, чтобы максимальный элемент оказался на своём месте
            flip(arr, size);
        }
    }
}


// Stooge Sort
void stoogeSort(vector<int>& arr, int l, int r) {
    if (arr[l] > arr[r]) {
        swap(arr[l], arr[r]);
    }


    if (r - l > 1) {
        int t = (r - l + 1) / 3;


        // Рекурсивно сортировать первую 2/3, последние 2/3 и снова первую 2/3
        stoogeSort(arr, l, r - t);
        stoogeSort(arr, l + t, r);
        stoogeSort(arr, l, r - t);
    }
}


// Функция для генерации случайного массива
vector<int> generateRandomArray(int size, int minVal = 1, int maxVal = 100) {
    vector<int> arr(size);
    for (int i = 0; i < size; ++i) {
        arr[i] = rand() % (maxVal - minVal + 1) + minVal;
    }
    return arr;
}


// Функция для вывода массива
void printArray(const vector<int>& arr) {
    for (int x : arr) {
        cout << x << " ";
    }
    cout << endl;
}


// Основная программа с тестами
int main() {
    srand(time(0)); // Инициализация генератора случайных чисел


    // Размеры массивов для тестов
    vector<int> sizes = {10, 100};


    for (int size : sizes) {
        cout << "Тестирование на массиве длинной: " << size << endl;


        // Генерация случайного массива
        vector<int> originalArray = generateRandomArray(size);


        // Odd-Even Sort
        vector<int> arr1 = originalArray;
        cout << "Изначальный массив: ";
        printArray(arr1);
        oddEvenSort(arr1);
        cout << "Odd-Even Sort: ";
        printArray(arr1);


        // Pancake Sort
        vector<int> arr2 = originalArray;
        pancakeSort(arr2);
        cout << "Pancake Sort: ";
        printArray(arr2);


        // Stooge Sort
        vector<int> arr3 = originalArray;
        stoogeSort(arr3, 0, arr3.size() - 1);
        cout << "Stooge Sort: ";
        printArray(arr3);
        
        cout << "-------------------------------------" << endl;
        if (arr3 == arr2 and arr3 == arr1) {
            cout << "Все алгоритмы отработали успешно ";
        }
    }


    return 0;
}
