def _is_ascending(list):
    previous = list[0]
    for number in list:
        if previous > number:
            return False
        previous = number
    return True


def _find_boundary_indices(array, value):
    assert len(array) > 0
    lower_index = None
    lower_min_difference = -1
    upper_index = None
    upper_min_difference = -1
    for i in range(len(array)):
        assert array[i] != value
        if array[i] < value:
            if lower_min_difference > value - array[i] or lower_min_difference < 0:
                lower_index = i
                lower_min_difference = value - array[i]
        if array[i] > value:
            if upper_min_difference > array[i] - value or upper_min_difference < 0:
                upper_index = i
                upper_min_difference = array[i] - value
    return lower_index, upper_index


def match(A, B, max_error):
    if not _is_ascending(A):
        raise ValueError("match: got unsorted A")
    if not _is_ascending(B):
        raise ValueError("match: got unsorted B")

    matching_results = list()
    start_index_b = 0
    for index_a in range(len(A)):
        while A[index_a] - B[start_index_b] > max_error:
            start_index_b += 1
            if start_index_b == len(B):
                break
        if start_index_b == len(B):
            break
        index_b = start_index_b
        while B[index_b] - A[index_a] <= max_error:
            matching_results.append((abs(A[index_a] - B[index_b]), index_a, index_b))
            index_b += 1
            if index_b == len(B):
                break

    if len(matching_results) == 0:
        return list(), list()
    matching_results.sort()

    matched_indices_a = set()
    matched_indices_b = set()
    indices_a = list()
    indices_b = list()
    for matching_result in matching_results:
        index_a = matching_result[1]
        index_b = matching_result[2]
        if (index_a in matched_indices_a) or (index_b in matched_indices_b):
            continue
        if len(indices_a) > 0:
            lower_index, upper_index = _find_boundary_indices(indices_a, index_a)
            lower_B_condition = True
            upper_B_condition = True
            if lower_index is not None:
                lower_B_condition = indices_b[lower_index] < index_b
            if upper_index is not None:
                upper_B_condition = indices_b[upper_index] > index_b
            if not lower_B_condition or not upper_B_condition:
                continue
        matched_indices_a.add(index_a)
        matched_indices_b.add(index_b)
        indices_a.append(index_a)
        indices_b.append(index_b)

    indices_a, indices_b = map(list, zip(*sorted(zip(indices_a, indices_b))))
    assert _is_ascending(indices_a) and _is_ascending(indices_b)

    return indices_a, indices_b