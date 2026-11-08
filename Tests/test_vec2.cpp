#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "SceneData/Vec2.h"

using Catch::Approx;

TEST_CASE("Vec2 init")
{
    Vec2 v{};
    CHECK(v.x == 0.0f);
    CHECK(v.y == 0.0f);
}

TEST_CASE("Vec2 arithmetic")
{
    Vec2 a{1, 2}, b{3, -4};
    CHECK((a + b).x == 4.0f);
    CHECK((a + b).y == -2.0f);
    CHECK((a - b).x == -2.0f);
    CHECK((a - b).y == 6.0f);
    CHECK((a * 2.0f).x == 2.0f);
    CHECK((a * 2.0f).y == 4.0f);

    a += b;
    CHECK(a.x == 4.0f); 
    CHECK(a.y == -2.0f);
}

TEST_CASE("Vec2 dot and cross")
{
    Vec2 x{1, 0}, y{0, 1};
    CHECK(dot(x, y) == 0.0f);
    CHECK(dot(Vec2{2, 3}, Vec2{4, 5}) == 23.0f);
    CHECK(cross(x, y) == 1.0f); // +y, cw
    CHECK(cross(y, x) == -1.0f);
    CHECK(cross(x, x) == 0.0f);
}