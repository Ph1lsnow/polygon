class Solution {
private:
    static constexpr int MAX_DIGITS = 11;
    static constexpr int MASK_SIZE = 1 << 10;
    int dp[MAX_DIGITS][2][MASK_SIZE];
    int dfs(int index, int usedMask, bool isTight, const string &digits) {
        if (index == static_cast<int>(digits.size())) {
            return usedMask == 0 ? 0 : 1;
        }
        if (dp[index][isTight][usedMask] != -1) {
            return dp[index][isTight][usedMask];
        }
        int limit = isTight ? (digits[index] - '0') : 9;
        int result = 0;

        for (int digit = 0; digit <= limit; ++digit) {
            if (usedMask == 0 && digit == 0) {
                result += dfs(index + 1, usedMask, isTight && (digit == limit), digits);
            } else {
                if ((usedMask & (1 << digit)) == 0) {
                    result += dfs(index + 1, usedMask | (1 << digit), isTight && (digit == limit), digits);
                }
            }
        }

        dp[index][isTight][usedMask] = result;
        return result;
    }

public:
    int numDupDigitsAtMostN(int n) {
        memset(dp, -1, sizeof(dp));
        string digits = to_string(n);
        return n - dfs(0, 0, true, digits);
    }