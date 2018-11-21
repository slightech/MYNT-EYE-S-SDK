#import "CVGLView.h"
#import "AppDelegate.h"




#define VVMINX(r) ((r.size.width>=0) ? (r.origin.x) : (r.origin.x+r.size.width))
#define VVMAXX(r) ((r.size.width>=0) ? (r.origin.x+r.size.width) : (r.origin.x))
#define VVMINY(r) ((r.size.height>=0) ? (r.origin.y) : (r.origin.y+r.size.height))
#define VVMAXY(r) ((r.size.height>=0) ? (r.origin.y+r.size.height) : (r.origin.y))
#define VVMIDX(r) (r.origin.x+(r.size.width/2.0))
#define VVMIDY(r) (r.origin.y+(r.size.height/2.0))
#define GLDRAWTEXQUADMACRO(texName,texTarget,texFlipped,src,dst)										\
{																										\
	GLuint		localMacroTexTarget=texTarget;															\
	NSRect		localMacroSrc=src;																		\
	NSRect		localMacroDst=dst;																		\
	BOOL		localMacroFlip=texFlipped;																\
	GLfloat		vvMacroVerts[]={																		\
		VVMINX(localMacroDst), VVMINY(localMacroDst), 0.0,												\
		VVMAXX(localMacroDst), VVMINY(localMacroDst), 0.0,												\
		VVMAXX(localMacroDst), VVMAXY(localMacroDst), 0.0,												\
		VVMINX(localMacroDst), VVMAXY(localMacroDst), 0.0};												\
	GLfloat		vvMacroTexs[]={																			\
		VVMINX(localMacroSrc),	(localMacroFlip ? VVMAXY(localMacroSrc) : VVMINY(localMacroSrc)),		\
		VVMAXX(localMacroSrc),	(localMacroFlip ? VVMAXY(localMacroSrc) : VVMINY(localMacroSrc)),		\
		VVMAXX(localMacroSrc),	(localMacroFlip ? VVMINY(localMacroSrc) : VVMAXY(localMacroSrc)),		\
		VVMINX(localMacroSrc),	(localMacroFlip ? VVMINY(localMacroSrc) : VVMAXY(localMacroSrc))};		\
	glVertexPointer(3,GL_FLOAT,0,vvMacroVerts);															\
	glTexCoordPointer(2,GL_FLOAT,0,vvMacroTexs);														\
	glBindTexture(localMacroTexTarget,texName);															\
	glDrawArrays(GL_QUADS,0,4);																			\
	glBindTexture(localMacroTexTarget,0);																\
}




@implementation CVGLView


+ (NSRect) rectThatFitsRect:(NSRect)a inRect:(NSRect)b	{
	NSRect		returnMe = NSMakeRect(0,0,0,0);
	double		bAspect = b.size.width/b.size.height;
	double		aAspect = a.size.width/a.size.height;
	
	//	if the rect i'm trying to fit stuff *into* is wider than the rect i'm resizing
	if (bAspect > aAspect)	{
		returnMe.size.height = b.size.height;
		returnMe.size.width = returnMe.size.height * aAspect;
	}
	//	else if the rect i'm resizing is wider than the rect it's going into
	else if (bAspect < aAspect)	{
		returnMe.size.width = b.size.width;
		returnMe.size.height = returnMe.size.width / aAspect;
	}
	else	{
		returnMe.size.width = b.size.width;
		returnMe.size.height = b.size.height;
	}
	returnMe.origin.x = (b.size.width-returnMe.size.width)/2.0+b.origin.x;
	returnMe.origin.y = (b.size.height-returnMe.size.height)/2.0+b.origin.y;
	
	return returnMe;
}


- (id) initWithFrame:(NSRect)f	{
	if (self = [super initWithFrame:f])	{
		renderLock = OS_SPINLOCK_INIT;
		initialized = NO;
		return self;
	}
	[self release];
	return nil;
}
- (id) initWithCoder:(NSCoder *)c	{
	if (self = [super initWithCoder:c])	{
		renderLock = OS_SPINLOCK_INIT;
		initialized = NO;
		return self;
	}
	[self release];
	return nil;
}


- (void) drawRect:(NSRect)r	{
	OSSpinLockLock(&renderLock);
	if (!initialized)	{
		NSOpenGLContext			*sharedCtx = [appDelegate sharedContext];
		NSOpenGLPixelFormat		*pFmt = [appDelegate pixelFormat];
		
		NSOpenGLContext			*newCtx = [[NSOpenGLContext alloc] initWithFormat:pFmt shareContext:sharedCtx];
		[self setOpenGLContext:newCtx];
		[newCtx setView:self];
		initialized = YES;
	}
	OSSpinLockUnlock(&renderLock);
}
- (void) drawTextureRef:(CVOpenGLTextureRef)n	{
	if (n==nil)
		return;
	//NSLog(@"%s",__func__);
	
	OSSpinLockLock(&renderLock);
	
	CGLContextObj	cgl_ctx = [[self openGLContext] CGLContextObj];
	GLuint			name = CVOpenGLTextureGetName(n);
	GLenum			target = CVOpenGLTextureGetTarget(n);
	BOOL			flipped = (CVOpenGLTextureIsFlipped(n)) ? YES : NO;
	GLfloat			bl[2], br[2], tl[2], tr[2];
	CVOpenGLTextureGetCleanTexCoords(n, bl, br, tr, tl);
	NSRect			texRect = NSMakeRect(bl[0], bl[1], br[0]-bl[0], tr[1]-br[1]);
	if (texRect.size.height < 0)
		texRect = NSMakeRect(bl[0], tl[1], br[0]-bl[0], br[1]-tr[1]);
	//NSLog(@"\t\ttexRect is (%0.2f, %0.2f) : %0.2f x %0.2f",texRect.origin.x,texRect.origin.y,texRect.size.width,texRect.size.height);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glHint(GL_CLIP_VOLUME_CLIPPING_HINT_EXT, GL_FASTEST);
	glDisable(GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	
	glEnable(target);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	
	//	set up the view to draw
	NSRect				bounds = [self bounds];
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, (GLsizei) bounds.size.width, (GLsizei) bounds.size.height);
	glOrtho(bounds.origin.x, bounds.origin.x+bounds.size.width, bounds.origin.y, bounds.origin.y+bounds.size.height, 1.0, -1.0);
	glDisable(GL_BLEND);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	//	clear the view
	glClearColor(0.0,0.0,0.0,0.0);
	glClear(GL_COLOR_BUFFER_BIT);
	
	//	determine the rect that i need to draw the texture in
	NSRect			dstRect = [CVGLView rectThatFitsRect:texRect inRect:bounds];
	GLDRAWTEXQUADMACRO(name,target,flipped,texRect,dstRect);
	
	glFlush();
	
	OSSpinLockUnlock(&renderLock);
}


@end
