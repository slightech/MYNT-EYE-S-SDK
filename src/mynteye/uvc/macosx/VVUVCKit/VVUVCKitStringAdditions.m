#import "VVUVCKitStringAdditions.h"

@implementation NSString (VVUVCKitStringAdditions)

- (BOOL) containsString:(NSString *)n	{
	BOOL		returnMe = NO;
	if (n != nil)	{
		NSRange		foundRange = [self rangeOfString:n];
		if (foundRange.location!=NSNotFound && foundRange.length==[n length])
			returnMe = YES;
	}
	return returnMe;
}

@end
