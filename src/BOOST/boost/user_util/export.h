#pragma once

#ifdef BOOST_USER_EXPORTS
#define BOOST_USER_API __declspec(dllexport)
#else
#define BOOST_USER_API __declspec(dllimport)
#endif