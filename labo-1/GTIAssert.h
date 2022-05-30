#pragma once

/**
 * @file Assert.h
 *
 * @brief D�finit l'assertion pour une configuration en mode debug
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

// Visual Studio d�finit automatiquement "_DEBUG" lors d'une compilation en mode debug. On accepte �galement la d�finition plus g�n�rique "DEBUG"
#if defined(_DEBUG) || defined(DEBUG)

#include <cassert>
#include <stdio.h>

// Assert avec un message, mais aucun formatage 
#define ASSERT(expression, message)             \
{                                               \
	if (!(expression))                          \
	{                                           \
		printf(message);                        \
		printf("\n");                           \
		assert(expression);                     \
	}                                           \
}

// Assert avec un message et du formatage
#define ASSERTF(expression, message, ...)       \
{                                               \
	if (!(expression))                          \
	{                                           \
		printf(message, __VA_ARGS__);           \
		printf("\n");                           \
		assert(expression);                     \
	}                                           \
}												
#else
#define ASSERT(expression, message)
#define ASSERTF(expression, message, ...)
#endif
