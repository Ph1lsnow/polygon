class Solution {
public:
    /*
        Сложность по времени: O(n)
        Сложность по памяти: O(n)
    */
    int findMinMoves(vector<int>& machines) {
        // 1) Подсчёт суммы всех платьев — O(n) по времени
        long long total = 0;
        for (int d : machines) {
            total += d;  // Проходим по всем элементам
        }
        
        int n = (int)machines.size();

        // 2) Проверка: если общее число платьев не делится без остатка на n, уравнять невозможно
        // Операция проверки делимости — O(1)
        if (total % n != 0) {
            return -1;
        }

        // Среднее количество платьев, которое должно быть в каждой машине
        long long avg = total / n;

        // 3) Создаём массив префиксных сумм left — O(n) по времени, O(n) по памяти
        // left[i] = сумма элементов machines[0..i-1]
        vector<long long> left(n, 0);
        for (int i = 1; i < n; ++i) {
            left[i] = left[i - 1] + machines[i - 1];
        }
        
        // 4) Создаём массив суффиксных сумм right — O(n) по времени, O(n) по памяти
        // right[i] = сумма элементов machines[i+1..n-1]
        vector<long long> right(n, 0);
        for (int i = n - 2; i >= 0; --i) {
            right[i] = right[i + 1] + machines[i + 1];
        }
        
        // 5) Для каждой машины вычисляем, сколько «перекачать» слева и справа — O(n)
        //    и берём максимум
        int ans = 0;
        for (int i = 0; i < n; ++i) {
            // Сколько не хватает «слева», чтобы привести машины 0..i-1 к идеалу
            long long L = max(0LL, (long long)i * avg - left[i]);
            // Сколько не хватает «справа» для машин i+1..n-1
            long long R = max(0LL, (long long)(n - i - 1) * avg - right[i]);
            
            // Максимум (L + R) за все i даёт ответ
            ans = max(ans, (int)(L + R));
        }
        
        // 6) Возвращаем итоговый результат — O(1)
        return ans;
    }
};
