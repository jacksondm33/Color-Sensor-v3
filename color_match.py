# Copyright (c) 2020 REV Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of REV Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
from wpilib import Color
from color_match_result import ColorMatchResult


class ColorMatch:

    __K_DEFAULT_CONFIDENCE = 0.95

    def __init__(self):
        self.__m_confidence_level = self.__K_DEFAULT_CONFIDENCE
        self.__m_colors_to_match = list()

    def add_color_match(self, color: Color) -> None:
        self.__m_colors_to_match.append(color)

    def set_confidence_threshold(self, confidence: float) -> None:
        if confidence < 0:
            confidence = 0
        elif confidence > 1:
            confidence = 1
        self.__m_confidence_level = confidence

    def match_color(self, color_to_match: Color) -> ColorMatchResult:
        match = self.match_closest_color(color_to_match)
        if match.confidence > self.__m_confidence_level:
            return match
        return None

    def match_closest_color(self, color: Color) -> ColorMatchResult:
        magnitude = color.red + color.blue + color.green
        if magnitude > 0.0 and len(self.__m_colors_to_match) > 0:
            normalized = Color(color.red / magnitude, color.green / magnitude,
                               color.blue / magnitude)
            min_distance = 1.0
            idx = 0
            for i in range(len(self.__m_colors_to_match)):
                target_distance = self.__calculate_distance(
                    self.__m_colors_to_match[i], normalized)
                if target_distance < min_distance:
                    min_distance = target_distance
                    idx = i
            match = ColorMatchResult(self.__m_colors_to_match[idx],
                                     1.0 - min_distance)
            return match
        return ColorMatchResult(Color.kBlack, 0.0)

    @classmethod
    def __calculate_distance(cls, color1: Color, color2: Color) -> float:
        red_diff = color1.red - color2.red
        green_diff = color1.green - color2.green
        blue_diff = color1.blue - color2.blue
        return math.sqrt((red_diff * red_diff + green_diff * green_diff +
                          blue_diff * blue_diff) / 2)

    @classmethod
    def make_color(cls, r: float, g: float, b: float) -> Color:
        return Color(r, g, b)
