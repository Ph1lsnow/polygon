import numpy as np

def count_and_replace(array):
    counts = np.bincount(array)
    return counts[array]

# Тестовые случаи

def test_count_and_replace():
    test_cases = [
        {"input": np.array([3, 3, 2, 1, 1, 1]), "expected": np.array([2, 2, 1, 3, 3, 3])},
        {"input": np.array([0, 0, 0, 4, 4, 2]), "expected": np.array([3, 3, 3, 2, 2, 1])},
        {"input": np.array([7, 7, 8, 8, 8, 9]), "expected": np.array([2, 2, 3, 3, 3, 1])},
        {"input": np.array([5]), "expected": np.array([1])},
        {"input": np.array([6, 6, 6, 6, 6]), "expected": np.array([5, 5, 5, 5, 5])},
    ]

    for i, test_case in enumerate(test_cases, start=1):
        input_array = test_case["input"]
        expected_output = test_case["expected"]
        result = count_and_replace(input_array)
        assert np.array_equal(result, expected_output), f"Тест {i} не пройден: ожидалось {expected_output}, получено {result}"
        print(f"Тест {i} пройден: {result}")

if __name__ == "__main__":
    test_count_and_replace()