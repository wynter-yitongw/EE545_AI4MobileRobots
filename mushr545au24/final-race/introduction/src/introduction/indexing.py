from introduction.fibonacci import compute_fibonacci


def extract_fibonacci_rows(data):
    """Extract the rows of a numpy array that correspond to the Fibonacci numbers,
    using integer array indexing.

    >>> data = np.array([[ 4,  7,  8],
    ...                  [14, 17, 18],
    ...                  [24, 27, 28],
    ...                  [34, 37, 38]])
    >>> extract_fibonacci_rows(data)
    array([[ 4,  7,  8],
           [14, 17, 18],
           [14, 17, 18],
           [24, 27, 28],
           [34, 37, 38]])
    """
    # BEGIN QUESTION 3.1
    max_idx = data.shape[0] - 1
    fib_rows = []
    i = 0
    while True:
        fib_num = compute_fibonacci(i)
        if fib_num > max_idx:
            break
        fib_rows.append(fib_num)
        i += 1
    return data[fib_rows]
    # END QUESTION 3.1


def increment_rows_with_odd_first_element(data):
    """Add one to rows of a numpy array where the 0th column element is odd,
    using Boolean array indexing.

    NOTE: This function does not have a return value: your implementation
    should modify the data argument in-place.

    >>> data = np.array([[ 0,  1,  2],
    ...                  [ 3,  4,  5],
    ...                  [ 6,  7,  8],
    ...                  [ 9, 10, 11],
    ...                  [12, 13, 14],
    ...                  [15, 16, 17]])
    >>> increment_rows_with_odd_first_element(data)
    >>> data
    array([[ 0,  1,  2],
           [ 4,  5,  6],
           [ 6,  7,  8],
           [10, 11, 12],
           [12, 13, 14],
           [16, 17, 18]])
    """
    # BEGIN QUESTION 3.2
    turn_odd = data[:, 0] % 2 == 1
    data[turn_odd] += 1
    # END QUESTION 3.2
