from statistics import mean, pstdev, pvariance

def read_file(fname):
    with open(fname) as f:
        array = [int(line) for line in f]
    return array

# Test 1
print('*'*10 + ' Test 1 ' + '*'*10)
arr1 = read_file('test1.txt')
print('Num samples:', len(arr1))
print('MIN:', min(arr1))
print('MAX:', max(arr1))
print('Average:', mean(arr1))
print('Standard deviation:', pstdev(arr1))
print('Variance:', pvariance(arr1))
print()

# Test 2
print('*'*10 + ' Test 2 ' + '*'*10)
arr1 = read_file('test2.txt')
print('Num samples:', len(arr1))
print('MIN:', min(arr1))
print('MAX:', max(arr1))
print('Average:', mean(arr1))
print('Standard deviation:', pstdev(arr1))
print('Variance:', pvariance(arr1))
print()

# Test 3
print('*'*10 + ' Test 3 ' + '*'*10)
arr1 = read_file('test3.txt')
print('Num samples:', len(arr1))
print('MIN:', min(arr1))
print('MAX:', max(arr1))
print('Average:', mean(arr1))
print('Standard deviation:', pstdev(arr1))
print('Variance:', pvariance(arr1))
print()

# All
print('*'*10 + ' All together ' + '*'*10)
arr1 = read_file('test1.txt')
arr2 = read_file('test2.txt')
arr3 = read_file('test3.txt')
arr = arr1 + arr2 + arr3
print('Num samples:', len(arr))
print('MIN:', min(arr))
print('MAX:', max(arr))
print('Average:', mean(arr))
print('Standard deviation:', pstdev(arr))
print('Variance:', pvariance(arr))
print()

