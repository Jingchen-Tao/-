import cv2 as cv
import numpy as np

img = cv.imread('p1.jpg',0)

cv.imshow('窗口名称',img)
cv.waitKey(0)
cv.destroyWindow('窗口名称')

"""
Given two integer arrays nums1 and nums2, return an array of their intersection. Each element in the result must be unique and you may return the result in any order.

 

Example 1:

Input: nums1 = [1,2,2,1], nums2 = [2,2]
Output: [2]
Example 2:

Input: nums1 = [4,9,5], nums2 = [9,4,9,8,4]
Output: [9,4]
Explanation: [4,9] is also accepted.
 

Constraints:

1 <= nums1.length, nums2.length <= 1000
0 <= nums1[i], nums2[i] <= 1000
"""

def binary_search(target: int, nums: list[int]) -> int | bool:
    l, r = 0, len(nums) - 1
    while l <= r:
        mid = (l + r) // 2
        if nums[mid] == target:
            return mid
        elif nums[mid] > target:
            r = mid - 1          # 往左半边找
        else:                     # nums[mid] < target
            l = mid + 1          # 往右半边找
    return False


class Solution:
    def isPerfectSquare(self, num: int) -> bool:

        l=0
        r=num
        while l<=r:
            mid=(l+r)//2
            if mid*mid == num :
                return True
            elif mid*mid<num:
                l=mid+1
            else:
                r=mid-1

        return False

class Solution:
    def searchRange(self, nums: List[int], target: int) -> List[int]:
        l=0
        r=len(nums)-1
        while l<=r:
            mid=(l+r)//2
            if nums[mid]==target:
                r=mid
                l=mid
                while nums[l-1]==nums[l]:
                    l-=1

                while nums[r+1]==nums[r]:
                    r+=1

                return [l,r]
            elif nums[mid] >= target:
                r=mid-1
            else:
                l=mid+1

        return[-1,-1]