#import "VVUVCUIElement.h"




@implementation VVUVCUIElement


- (id) initWithFrame:(NSRect)f	{
	if (self = [super initWithFrame:f])	{
		delegate = nil;
		enabled = YES;
		valSlider = nil;
		valField = nil;
		val = 0;
		min = 0;
		max = 0;
		//[[self titleCell] setControlSize:NSMiniControlSize];
		[self setTitleFont:[NSFont systemFontOfSize:9]];
		[self setBorderType:NSNoBorder];
		[self setBoxType:NSBoxSecondary];
		//[self setTitle:@"TestTitle"];
		NSView			*contentView = [self contentView];
		
		valSlider = [[NSSlider alloc] initWithFrame:NSMakeRect(0,0,200,20)];
		[[valSlider cell] setControlSize:NSMiniControlSize];
		[valSlider setContinuous:YES];
		[valSlider setTarget:self];
		[valSlider setAction:@selector(uiItemUsed:)];
		
		valField = [[NSTextField alloc] initWithFrame:NSMakeRect(0,0,200,20)];
		//[[valField cell] setControlSize:NSMiniControlSize];
		[valField setFont:[NSFont systemFontOfSize:9]];
		NSNumberFormatter	*formatter = [[NSNumberFormatter alloc] init];
		[valField setFormatter:formatter];
		[formatter release];
		[valField setTarget:self];
		[valField setAction:@selector(uiItemUsed:)];
		
		[contentView addSubview:valSlider];
		[contentView addSubview:valField];
		[self _resizeContents];
		return self;
	}
	[self release];
	return nil;
}
- (void) dealloc	{
	if (valSlider != nil)	{
		[valSlider removeFromSuperview];
		[valSlider release];
		valSlider = nil;
	}
	if (valField != nil)	{
		[valField removeFromSuperview];
		[valField release];
		valField = nil;
	}
	[super dealloc];
}


- (void) setEnabled:(BOOL)n{
	if (enabled == n)
		return;
	enabled = n;
	if (enabled)	{
		[valSlider setEnabled:YES];
		[valField setEnabled:YES];
	}
	else	{
		[valSlider setEnabled:NO];
		[valField setEnabled:NO];
	}
}


- (void) _resizeContents	{
	//NSLog(@"%s",__func__);
	NSRect		contentBounds = [[self contentView] bounds];
	//NSLog(@"\t\tcontentBounds is (%f, %f) : %f x %f",contentBounds.origin.x,contentBounds.origin.y,contentBounds.size.width,contentBounds.size.height);
	NSRect		sliderRect;
	NSRect		txtRect = contentBounds;
	txtRect.size = NSMakeSize(50,16);
	sliderRect.size = NSMakeSize(contentBounds.size.width-txtRect.size.width-2, txtRect.size.height);
	sliderRect.origin = NSMakePoint(0,0);
	txtRect.origin = NSMakePoint(contentBounds.size.width-txtRect.size.width, 0);
	[valSlider setFrame:sliderRect];
	[valField setFrame:txtRect];
	//NSLog(@"\t\tslider rect is (%f, %f) : %f x %f",sliderRect.origin.x,sliderRect.origin.y,sliderRect.size.width,sliderRect.size.height);
	//NSLog(@"\t\ttext rect is (%f, %f) : %f x %f",txtRect.origin.x,txtRect.origin.y,txtRect.size.width,txtRect.size.height);
	[valSlider setAutoresizingMask:NSViewWidthSizable];
	[valField setAutoresizingMask:NSViewMinXMargin];
}
- (void) uiItemUsed:(id)sender	{
	//NSLog(@"%s",__func__);
	if (sender == valSlider)	{
		//	update the val, then update the val field
		val = [valSlider intValue];
		[valField setIntValue:val];
	}
	else if (sender == valField)	{
		//	update the val, then update the val slider
		val = [valField intValue];
		[valSlider setIntValue:val];
	}
	//	if there's a delegate, let it know that my val changed!
	if (delegate != nil)
		[delegate controlElementChanged:self];
}


@synthesize delegate;
- (void) setVal:(int)n	{
	//NSLog(@"%s ... %@, %d",__func__,[self title],n);
	val = n;
	[valField setIntValue:n];
	[valSlider setIntValue:n];
}
- (int) val	{
	return val;
}
- (void) setMin:(int)n	{
	//NSLog(@"%s ... %@, %ld",__func__,[self title],n);
	min = n;
	NSNumberFormatter		*fmt = [valField formatter];
	[fmt setMinimum:[NSNumber numberWithInt:n]];
	[valSlider setMinValue:n];
}
- (int) min	{
	return min;
}
- (void) setMax:(int)n	{
	//NSLog(@"%s ... %@, %ld",__func__,[self title],n);
	max = n;
	NSNumberFormatter		*fmt = [valField formatter];
	[fmt setMaximum:[NSNumber numberWithInt:n]];
	[valSlider setMaxValue:n];
}
- (int) max	{
	return max;
}


@end
