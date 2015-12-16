/*   Description: " Container for needed parameters for localization."
 *   Author: "Toni Rosinol"
 *   Date: "24.11.2015"
 */
#ifndef _PARAMETERS_BAG_
#define _PARAMETERS_BAG_

#include <string.h>
#include "topics_parameters.h"
#include "localizer_parameters.h"

// Parameters bag.
struct ParametersBag {
	TopicsParameters topics; ///< Topics parameters.
	LocalizerParameters localizer; ///< Localizer parameters.
};

#endif
