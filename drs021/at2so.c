/*
 * what: at2so - drone command relay (proxy)
 * who:  miru
 * when: April 2011...
 *
 * ARMTC = /home/arm-2010.09/bin/arm-none-linux-gnueabi
 * ARMCF =
 * ARMCF += -Wall
 * ARMCF += -Wstrict-prototypes
 * ARMCF += -Os
 * 
 * %.arm: %.c
 *	$(ARMTC)-gcc $(ARMCF) $< -o $@ -lm -lrt
 *	$(ARMTC)-strip $@
 */
#define	EN_SLOG	1
#if	EN_SLOG
char version[] = "@(#) at2so 0.21 20130723";
#endif

#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <time.h>
typedef struct timeval	tmv_t;
typedef struct timespec	tms_t;

#include <termios.h>
typedef struct termios	tio_t;

#include <netdb.h>
#include <netinet/in.h>
typedef union {
	struct sockaddr     sa;
	struct sockaddr_in  sin;
}	soa_t;
#define	sin_fmly	sin.sin_family
#define	sin_port	sin.sin_port
#define	sin_addr	sin.sin_addr.s_addr
#define	IPV4A(a,b,c,d)	htonl(((a&0xff)<<24)|((b&0xff)<<16)|((c&0xff)<<8)|(d&0xff))

typedef union { int i; float f; } vif_t;
typedef unsigned char	u08_t;
typedef unsigned short	u16_t;
typedef unsigned int	u32_t;

#define	NEL(x)		(int)(sizeof(x)/sizeof(x[0]))
#define	MAX(a,b)	((a)>(b)?(a):(b))
#define	MINMAX(n,x,v)	((v)<(n)?(n):(v)>(x)?(x):(v))
#define	DEG2RAD(d)	((double)(d)*M_PI/180.0)
#define	RAD2DEG(r)	((double)(r)*180.0/M_PI)

#define	gps_in(b)	SLOG(b)
#define	bmp_in(b)	SLOG(b)
#define	hmc_in(b)	SLOG(b)

/*
 * drone stuff needed...
 */
typedef struct { /* navdata header */
	u32_t	mag;
#define	NAVMAG	0x55667788
	u32_t	cst;
	/* according to SDK 2.1 */
#define	AS_EMERGENCY	(1<<31) /* 1 emergency landing */
#define	AS_COM_WATCHDOG	(1<<30) /* 1 communication watchdog trigger */
#define	AS_ADC_WATCHDOG	(1<<29) /* NA 1 delay in uart2 dsr (>5ms) */
#define	AS_CTL_WATCHDOG	(1<<28) /* NA 1 delay in control execution */
#define	AS_ACQ_THREAD	(1<<27) /* NA 1 acquisition thread on */
#define	AS_VID_THREAD	(1<<26) /* NA 1 video thread on */
#define	AS_NAV_THREAD	(1<<25) /* NA 1 navdata thread on */
#define	AS_AT__THREAD	(1<<24) /* NA 1 AT codec thread on */
#define	AS_PIC_V_OK	(1<<23) /* NA 1 PIC version ok */*/
#define	AS_CUTOUT	(1<<22) /* NA 1 motor emergency stop */
#define	AS_ULTRASOUND	(1<<21) /* 1 ultrasound sensor problem */
#define	AS_WIND		(1<<20) /* NA 1 too much wind */
#define	AS_ANGLES_OOR	(1<<19) /* NA 1 angles out of range */
#define	AS_MAG_CALRQ	(1<<18) /* NA 1 magnetometer needs calibration */
#define	AS_TIMER	(1<<17) /* NA 1 timer elapsed */
#define	AS_USER_ESTOP	(1<<16) /* NA 1 user emergency stop */
#define	AS_VBAT_LOW	(1<<15) /* 1 battery too low to fly */
#define	AS_SW_FAULT	(1<<14) /* NA 1 software fault */
#define	AS_COM_LOST	(1<<13) /* NA 1 communication problem */
#define	AS_MOTORS	(1<<12) /* NA 1 motor problem */
#define	AS_NAV_BOOT	(1<<11) /* NA 1 no navdata options */
#define	AS_NAV_DEMO	(1<<10) /* NA 1 only navdata demo */
#define	AS_USB		(1<< 9) /* NA 1 USB key ready */
#define	AS_TRAVELLING	(1<< 8) /* NA 1 travelling enable ? */
#define	AS_CAMERA	(1<< 7) /* NA 1 camera ready */
#define	AS_COMMAND_ACK	(1<< 6) /* 1 command ack */
#define	AS_USER_STRT	(1<< 5) /* NA 1 user start button on */
#define	AS_ALTCTRL	(1<< 4) /* 1 altitude control active */
#define	AS_ANGCTRL	(1<< 3) /* NA 1 angular speed control, 0 euler angle control */
#define	AS_VISION	(1<< 2) /* NA 1 vision enable */
#define	AS_VIDEO	(1<< 1) /* NA 1 video enable */
#define	AS_FLY		(1<< 0) /* 1 flying */
	u32_t	seq;
	u32_t	vdf;
}	__attribute__((packed)) navh_t;

typedef struct { /* navdata option header */
	short	tag;
	u16_t	siz;
}	__attribute__((packed)) noph_t;

typedef struct { /* tag -1 checksum */
	noph_t	hdr;
	u32_t	cks;
}	__attribute__((packed)) ntck_t;

typedef struct { /* tag 00 */
	noph_t	hdr;
	u32_t	stat;	/* ctrl_state */
	u32_t	cbat;	/* vbat_flying_percentage (23 bits) */
	float	theta;
	float	phi;
	float	psi;
	int	altitude;
	struct {
		float	vx;
		float	vy;
		float	vz;
		u32_t	num_frames;
		float	detection_camera[12];	/* SDK 1.5 calls it obsolete */
		u32_t	detection_tag_index;	/* SDK 1.5 calls it obsolete */
		int	detection_camera_type;
		float	drone_camera[12];	/* SDK 1.6 calls it obsolete */
	}	trash;
}	__attribute__((packed)) nt00_t;

typedef struct { /* tag 25 (navdata_hdvideo_stream_t) */
	noph_t	hdr;
	u32_t	hdvideo_state;
#define	HDVIDEO_STORAGE_FIFO_IS_FULL (1<< 0)
#define	HDVIDEO_USBKEY_IS_PRESENT    (1<< 8)
#define	HDVIDEO_USBKEY_IS_RECORDING  (1<< 9)
#define	HDVIDEO_USBKEY_IS_FULL       (1<<10)
	u32_t	storage_fifo_nb_packets;
	u32_t	storage_fifo_size;
	u32_t	usbkey_size;
	u32_t	usbkey_freespace;
	u32_t	frame_number;
	u32_t	usbkey_remaining_time;	/* in seconds */
}	__attribute__((packed)) nt25_t;

static int strequ(char *s, char *d)
{
	while (1) if (*s != *d++) break; else if (*s++ == 0) return 1;
	return 0;
}

static int strbeg(char *hdr, char *str)
{
	while (*hdr) if (*hdr == *str) hdr++, str++; else return 0;
	return 1;
}

static void strtxf(char *s, char *d, int n)
{
	while (--n >= 0 && (*d = *s++)) d++;
}

static int cmd2av(char *s, char **av, int ma)
{
	int	n = 0, i;

	for (av[n++] = s; *s && *s != '=' && *s != ','; s++);
	if (*s)
		for (i = 0, *s++ = 0, av[n++] = s; *s && n < ma; s++)
			if (*s == '\"') i ^= 01;
			else if (!i && *s == ',') *s = 0, av[n++] = s + 1;
	/* take out quotes */
	for (i = 0; i < n; i++)
		if (*(s = av[i]) == '\"') {
			for (av[i] = ++s; *s; s++);
			if (*(--s) == '\"') *s = 0;
		}
	return n;
}

/*
 *  0 BLINK_GREEN_RED
 *  1 BLINK_GREEN
 *  2 BLINK_RED
 *  3 BLINK_ORANGE
 *  4 SNAKE_GREEN_RED
 *  5 FIRE
 *  6 STANDARD
 *  7 RED
 *  8 GREEN
 *  9 RED_SNAKE
 * 10 BLANK
 * 11 RIGHT_MISSILE
 * 12 LEFT_MISSILE
 * 13 DOUBLE_MISSILE
 * 14 FRONT_LEFT_GREEN_OTHERS_RED
 * 15 FRONT_RIGHT_GREEN_OTHERS_RED
 * 16 REAR_RIGHT_GREEN_OTHERS_RED
 * 17 REAR_LEFT_GREEN_OTHERS_RED
 * 18 LEFT_GREEN_RIGHT_RED
 * 19 LEFT_RED_RIGHT_GREEN
 * 20 BLINK_STANDARD
 */
#define	LED_BLINK_GREEN_RED	"LED=,0,1084227584"
#define	LED_BLINK_GREEN		"LED=,1,1084227584"
#define	LED_BLINK_RED		"LED=,2,1084227584"
#define	LED_BLINK_ORANGE	"LED=,3,1084227584"

/* configuration mechanism */
typedef struct {
	char	var[32];
	char	val[36];
}	cvv_t;

typedef struct {
	int	cvvr;
	int	cvvw;
	cvv_t	cvvb[32];
}	cfg_t;

/*
 * global variables
 */
struct {
	int	exit;
	tms_t	tref;	/* reference time for tnow */
	u32_t	tnow;
	u32_t	flag;	/* program flags */
#define	F_BLK_WIFI	0x00000001

	soa_t	dcmd; /* drone cmd port on network */
	soa_t	dnav; /* drone nav port on network */
	soa_t	ddat; /* drone dat port on network */

	fd_set	sd;
	int	md;
	int	cd; soa_t ca;	/* cmd socket, sends to port 5556 */
	int	id; soa_t ia;	/* proxy dat socket, outside gets redirected here */
	int	fd; soa_t fa;	/* proxy dat sender socket (transient) */
	int	dd; soa_t da;	/* proxy dat receiver, drone is told to send here */
	int	nd; soa_t na;	/* proxy nav socket, outside gets redirected here */
	int	rd; soa_t ra;	/* proxy nav receiver, drone is told to send here */
	int	ad; tio_t ap[2];/* Arduino serial port */
	int	ld;		/* log file desciptor if used */
	char	lfn[64];	/* log file name */

	struct ardu {
		u32_t	trcv;	/* tnow when last message was received */
		u32_t	tout;	/* sequencer timeout */
		int	swdg;	/* status watchdog sequencer */
		int	scfg;	/* status configuration sequencer */
		int	rcfg;	/* configuration sequencer retry count */
		int	acfg;	/* Arduino wants ACK when configs are done */
		cfg_t	cfgs;	/* configuration command queue */
		int	cdrq;	/* idev control data dump request */
		u32_t	sdto;	/* Arduino data transmission timeout */
		int	link;	/* serial link status */
		struct {	/* user steering commands */
			int	mode;
			vif_t	rpgy[4];
		}	usr;
	}	ardu;

	struct idev {
		u32_t	trcv;	/* tnow when last message was received */
		soa_t	fnav;	/* forwarding address for navdata */
		int	nall;	/* forward all Navdata, header only otherwise */
		u32_t	fcst;	/* AS_COM_WATCHDOG, AS_COMMAND_ACK for iDev */
		char	sid[32];/* session ID arg */
		char	sds[64];/* session descriptor */
		char	pid[32];/* profile ID arg */
		char	aid[32];/* applic  ID arg */
	}	idev;

	/* drone */
	u32_t	ref;	/* ATREF command */
#define	REF_LAND	0x11540000
#define	REF_STRT	0x11540200
#define	REF_ESTP	0x11540100
	u32_t	seq;	/* AT command sequence number */
	int	nat;	/* AT command buffer fill (at_send()) */
	char	atb[1024];
	char	dfmw[16]; /* drone firmware version */
	int	ard2;	/* is AR.drone 2.0 */
	int	vzap;	/* current video zap */
	int	rvid;	/* D2, record flight video on USB stick */
	int	anim;	/* animation enable */
	int	lgid;	/* log index for log file name */
	u32_t	usbc;	/* D2 USB stick capacity in 1K blocks */
	u32_t	usbf;	/* D2 USB stick free space in 1K blocks */
	navh_t	navh;	/* last navdata header */
	u32_t	ccst;	/* changed bits in navh.cst */
	struct {	/* last tag 0 extracts */
		int	bat;	/* battery percentage */
		int	psi;	/* heading in [deg * 1000] */
		int	alt;	/* altitude in D2 [mm], D1 [?] */
	}	nt00;

	/* custom CONFIGS for newer drone firmware... */
	int	idse;	/* enable insertion of CONFIG_IDS */
	char	*sid;	/* session id */
	char	*pid;	/* profile id */
	char	*aid;	/* application id */
	char	*sds;	/* session description */

	struct {	/* drone data port 5559 intercept */
		int	typ;	/* data type */
		int	nbu;
		char	buf[6*1024];
	}	dat;

	int	lgs;	/* current size of log file */
#define	MLGS	(4*1024*1024)
	char	lgb[512];	/* slog temporary buffer */
}	gl;

static void add_fd(int fd)
{
	if (fd < 0) return;
	FD_SET(fd,&gl.sd);
	if (fd >= gl.md) gl.md = fd + 1;
}

static void del_fd(int fd)
{
	if (fd < 0) return;
	FD_CLR(fd,&gl.sd);
	while (gl.md > 0 && !FD_ISSET(gl.md-1,&gl.sd)) gl.md--;
}

/* Linux time is an 8 byte entity which is a little large for here, using 'tics'
 * since start of program at a lower resolution to make it fit into 32 bit variable.
 * A u32_t with 1 ms resolution rolls after ~49.7 days	*/
#define	NTPS		1000
#define	S2TIC(s)	((int)(((double)(s))*NTPS+0.5))

#define	CLOCK_SEL	CLOCK_MONOTONIC

static int tref(void)
{
	return clock_gettime(CLOCK_SEL,&gl.tref);
}

static u32_t tnow(void)
{
	tms_t	tv;
	u32_t	tnow;

	clock_gettime(CLOCK_SEL,&tv);
#define	NSUBSEC 1000000000L
	tv.tv_sec -= gl.tref.tv_sec;
	if ((tv.tv_nsec -= gl.tref.tv_nsec) < 0) {
		tv.tv_sec--;
		tv.tv_nsec += NSUBSEC;
	}
	tnow = tv.tv_sec * NTPS + tv.tv_nsec / (NSUBSEC/NTPS);
	return tnow;
}

#if	EN_SLOG
static void slog_b(int n)
{
	int	k;
	char	buf[32+128];

	if (gl.ld < 0 || n < 0) return;
	if (n > 128) n = 128;
	k = write(gl.ld,buf,sprintf(buf,"%7u: %.*s\n",tnow(),n,gl.lgb));
	if (k <= 0 || (gl.lgs += k) >= MLGS) close(gl.ld), gl.ld = -2;
}

#define	SLOG(...)	slog_b(sprintf(gl.lgb,__VA_ARGS__))

static void slog_dcst(void)
{
	u32_t	m, c, s;

	s = gl.navh.cst;
	c = gl.ccst;
	if (c & (m = AS_EMERGENCY))    c &= ~m, SLOG("ESTP,%d",s&m?1:0);
	if (c & (m = AS_COMMAND_ACK))  c &= ~m, SLOG("CMAK,%d",s&m?1:0);
	if (c & (m = AS_COM_WATCHDOG)) c &= ~m, SLOG("WDOG,%d",s&m?1:0);
	if (c & (m = AS_MOTORS))       c &= ~m, SLOG("MOTO,%d",s&m?1:0);
	if (c & (m = AS_ANGLES_OOR))   c &= ~m, SLOG("AOOR,%d",s&m?1:0);
	if (c & (m = AS_CUTOUT))       c &= ~m, SLOG("COUT,%d",s&m?1:0);
	if (c & (m = AS_ALTCTRL))      c &= ~m, SLOG("ALTC,%d",s&m?1:0);
	if (c & (m = AS_ULTRASOUND))   c &= ~m, SLOG("USND,%d",s&m?1:0);
	if (c & (m = AS_WIND))         c &= ~m, SLOG("WIND,%d",s&m?1:0);
	if (c & (m = AS_VBAT_LOW))     c &= ~m, SLOG("BLOW,%d",s&m?1:0);
	if (c & (m = AS_USER_STRT))    c &= ~m, SLOG("STRT,%d",s&m?1:0);
	if (c & (m = AS_FLY))          c &= ~m, SLOG("AIRB,%d",s&m?1:0);
	if (c) SLOG("DCST,%08x,%08x",c,s);
}

#else
#define	SLOG(...)
#define slog_dcst()
#endif

/* configuration cmd fifo */
static void psh_cfg(char *var, char *val)
{
	cfg_t	*cfg = &gl.ardu.cfgs;
	cvv_t	*cvv;
	int	i;

	cvv = &cfg->cvvb[i = cfg->cvvw];
	bzero(cvv,sizeof(*cvv));
	strtxf(var,cvv->var,NEL(cvv->var)-1);
	strtxf(val,cvv->val,NEL(cvv->val)-1);
	if (++i >= NEL(cfg->cvvb)) i = 0;
	if (i == cfg->cvvw) return;
	cfg->cvvw = i;
}

static cvv_t *nxt_cfg(void)
{
	cfg_t	*cfg = &gl.ardu.cfgs;

	return cfg->cvvr == cfg->cvvw ? 0 : &cfg->cvvb[cfg->cvvr];
}

static void pop_cfg(void)
{
	cfg_t	*cfg = &gl.ardu.cfgs;
	int	i;

	if ((i = cfg->cvvr) == cfg->cvvw) return;
	cfg->cvvr = ++i >= NEL(cfg->cvvb) ? 0 : i;
}

/* AT cmd editor */
static void at_send(char *dat)
{
	char	*s, *b, *d, *p;
	int	k;

	d = gl.atb + gl.nat;
	for (s = dat; *s; s += *s ? 1 : 0) {
		for (b = s; *s && *s != '\r'; s++);			/* find end of command */
		p = d; *p++ = 'A'; *p++ = 'T'; *p++ = '*';		/* 'AT*' key prefix for drone */
		while (b < s && (*p++ = *b++) != '=');			/* copy <key>= */
		if ((k = (p - d) - 4) <= 0) continue;			/* <key> length */
		while (b < s && *b != ',') b++;				/* skip sequence number */
		if (k == 6 && d[3] == 'C' && d[5] == 'N' && gl.idse) {	/* need CONFIG_IDS */
			p--;
			p += sprintf(p,"_IDS=%u,\"%s\",\"%s\",\"%s\"\rAT*CONFIG=",++gl.seq,gl.sid,gl.pid,gl.aid);
		}
		p += sprintf(p,"%u",++gl.seq);	/* add local sequence number */
		while (b < s) *p++ = *b++;	/* copy arguments 'as is' */
		*p++ = '\r';
		d = p;
	}
	gl.nat = d - gl.atb;
}

static void nrq_in(void)
{
	soa_t	fa;
	int	n, dat[64];
	socklen_t fl;

	fl = sizeof(fa.sa);
	if ((n = recvfrom(gl.nd,dat,sizeof(dat),0,&fa.sa,&fl)) <= 0) return;
	SLOG("NRQI,%d,%d",n,dat[0]);
	if (n != 4) return;
	switch (dat[0]) {
	case 0: gl.idev.fnav.sin_port = 0; break;	/* missing on drone (would be nice for video) */
	case 1: gl.idev.fnav = fa; break;		/* navdata to specific socket request */
	}
}

static void *nav_tag(void *dat, int len, short tag, u16_t siz)
{
	union { u08_t *b; navh_t *h; noph_t *o; } p, e;

	e.b = (p.b = dat) + len;
	for (p.h++; p.b < e.b && p.o->siz >= sizeof(*p.o); p.b += p.o->siz)
		if (p.o->tag == tag && (siz == 0 || p.o->siz == siz)) return p.o;
	return 0;
}

static ntck_t *nav_ckt(void *dat, int len)
{
	union { u08_t *b; ntck_t *c; } p;

	if (len < (int)sizeof(*p.c)) return 0;
	p.b = dat; p.b += len; p.c--; /* checksum option is at end */
	if (p.c->hdr.siz != (u16_t)sizeof(*p.c)) return 0;
	if (p.c->hdr.tag != -1) return 0;
	return p.c;
}

#if	0
static int nav_del(void *dat, int len, short tag)
{
	union { u08_t *b; ntck_t *c; noph_t *o; } p, d, s;
	int	nd, k;

	if ((d.o = nav_tag(dat,len,tag,0)) == 0) return 0;
	if ((p.c = nav_ckt(dat,len)) == 0 || d.b >= p.b) return 0;
	nd = d.o->siz;
	for (s.b = d.b, k = nd; --k >= 0; s.b++) p.c->cks -= *s.b;
	for (p.c++, k = p.b - s.b; --k >= 0; *d.b++ = *s.b++);
	return nd;
}

static int nav_add(void *dat, int len, noph_t *nt)
{
	union { u08_t *b; ntck_t *c; noph_t *o; } s, p, e;

	if ((p.c = nav_ckt(dat,len)) == 0) return 0;
	if (nt->siz <= (int)sizeof(*nt) || nt->siz & 03) return 0;
	s.o = nt;
	e.b = p.b + nt->siz;
	*e.c = *p.c;
	while (p.b < e.b) e.c->cks += (*p.b++ = *s.b++);
	return nt->siz;
}
#endif

static navh_t *nav_dat(void *dat, int len, ntck_t **pck)
{
	navh_t	*nh;
	ntck_t	*ck;
	u08_t	*b;
	u32_t	cks;

	if (len < (int)(sizeof(navh_t)+sizeof(ntck_t))) return 0;
	if ((nh = dat)->mag != NAVMAG) return 0;
	if ((ck = nav_ckt(dat,len)) == 0) return 0; /* no checksum option */
	for (cks = 0, b = dat; b < (u08_t *)ck; b++) cks += *b;
	if (cks != ck->cks) return 0; /* bad checksum */
	if (pck) *pck = ck;
	return nh;
}

static int nav_addcks(void *dat, int len)
{
	u08_t	*b = dat;
	ntck_t	*ntck;
	int	i;

	ntck = (ntck_t *)&b[len];
	ntck->hdr.tag = -1;
	ntck->hdr.siz = sizeof(*ntck);
	ntck->cks = 0;
	for (i = len; --i >= 0; b++) ntck->cks += *b;
	return len + ntck->hdr.siz;
}

static void nh_setcst(navh_t *nh, ntck_t *ck)
{
	u08_t	*b = (u08_t *)&nh->cst;
	int	i;

	if (ck == 0) return;
	for (i = sizeof(nh->cst); --i >= 0; ) ck->cks -= *(b++);
	nh->cst &= ~(AS_COM_WATCHDOG|AS_COMMAND_ACK);
	nh->cst |= gl.idev.fcst;
	for (i = sizeof(nh->cst); --i >= 0; ) ck->cks += *(--b);
}

static void nav_in(void)
{
	navh_t	*nh;
	nt00_t	*nt00;
	nt25_t	*nt25;
	ntck_t	*ntck;
	int	n, len;
	u08_t	dat[1504];

	len = read(gl.rd,dat,sizeof(dat));
	if (len <= 0 || (nh = nav_dat(dat,len,&ntck)) == 0) return;	/* garbage */
	if ((gl.ccst = gl.navh.cst) == 0) gl.ccst = ~nh->cst;
	gl.navh = *nh;
	if ((gl.ccst ^= gl.navh.cst)) slog_dcst();
	if ((nt00 = nav_tag(dat,len,0,sizeof(*nt00)))) {
		n = (short)nt00->cbat;	/* 23 bit variable [22:15] same as [14] */
		if (gl.nt00.bat != n) gl.nt00.bat = n, SLOG("VBAT,%d",n);
		n = (int)nt00->psi;
		n = (n + (n < 0 ? -500 : +500))/1000;
		if (gl.nt00.psi != n) gl.nt00.psi = n, SLOG("HPSI,%d",n);
		n = nt00->altitude;
		if (gl.nt00.alt != n) gl.nt00.alt = n, SLOG("ALTI,%d",n);
	}
	else bzero(&gl.nt00,sizeof(gl.nt00));
	if (gl.ard2 && (nt25 = nav_tag(dat,len,25,sizeof(*nt25)))) {
		gl.usbc = nt25->usbkey_size;
		gl.usbf = nt25->usbkey_freespace;
	}

	if (gl.idev.fnav.sin_port) {
#if	0
		/* this tag is large, can't be turned off and has no use for us */
		len -= nav_del(dat,len,16);
#endif
		if (((nh->cst ^ gl.idev.fcst) & (AS_COM_WATCHDOG|AS_COMMAND_ACK))) nh_setcst(nh,ntck);
		if (gl.idev.nall == 0) len = nav_addcks(dat,sizeof(*nh));
		sendto(gl.rd,dat,len,0,&gl.idev.fnav.sa,sizeof(gl.idev.fnav.sa));
	}
}

static char *addhexp(void *p, int n, char *d)
{
	u08_t	*b = p;

	while (--n >= 0) d += sprintf(d,"%02X",*b++);
	return d;
}

typedef struct {	/* message for Arduino */
	u32_t	dcst;	/* drone status */
	char	cbat;	/* battery percentage left */
	char	sgps;	/* GPS fix status */
}	__attribute__ ((packed)) m2a_t;

static void ard_sq(void)
{
	struct ardu	*a = &gl.ardu;
	cvv_t		*cvv;
	char		*d, cmd[128];
	m2a_t		m2a;

	/* watchdog */
	switch (a->swdg) {
	case 3: a->swdg = 0;
		/* FALLTROUGH */
	case 0: if (a->trcv == 0) {
			write(gl.ad,"\002",1);
			SLOG("C_SC");
			a->tout = gl.tnow + S2TIC(0.1);
			a->swdg = 2;
			break;
		}
		else if (gl.navh.cst & AS_COM_WATCHDOG) {
			if ((gl.tnow - a->trcv) > S2TIC(1.0)) {
				a->link = -1;
				SLOG("LINK,%d",a->link);
				a->swdg = 4;
				break;
			}
			write(gl.ad,"\002",1);
			SLOG("W_SC");
			gl.seq = 0;
			at_send("COMWDG=");
			a->tout = gl.tnow + S2TIC(0.3);
			a->swdg = 1;
			a->scfg = 0;
		}
		break;
	case 1: if (!(gl.navh.cst & AS_COM_WATCHDOG) || gl.tnow >= a->tout) a->swdg = a->scfg = 0;
		break;
	case 2: if (a->trcv != 0 || gl.tnow > a->tout) a->swdg = 3;
		break;
	case 4: /* link failure */
		break;
	}

	/* config fifo */
	switch (a->scfg) {
	case 0: if (a->swdg) break;
		a->scfg = gl.navh.cst & AS_COMMAND_ACK ? 3 : 1;
		break;
	case 1: if ((cvv = nxt_cfg()) == 0) {
			if (a->acfg) {
				a->acfg = 0;
				at_send(LED_BLINK_GREEN",1");
				write(gl.ad,"\001",1);	/* send config ACK to Arduino */
			}
			if (a->cdrq == 0) break;
			/* iDev made a request for data (CTRL 4,6) */
			if (gl.dat.typ == 0) {
				gl.dat.typ = a->cdrq;
				gl.dat.nbu = 0;
				sprintf(cmd,"CTRL=,%d,0",gl.dat.typ);
				at_send(cmd);
				a->tout = gl.tnow + S2TIC(2.0);
				a->scfg = 5;
			}
			break;
		}
		sprintf(cmd,"CONFIG=,\"%s\",\"%s\"",cvv->var,cvv->val);
		SLOG("DCFG%s",cmd+7);
		at_send(cmd);
		a->tout = gl.tnow + S2TIC(0.5);
		a->scfg = 2;
		break;
	case 2: if (gl.navh.cst & AS_COMMAND_ACK) {
			pop_cfg();
			a->rcfg = 0;
			a->scfg = 3;
		}
		else if ((int)(gl.tnow - a->tout) >= 0) {
			a->scfg = 1;
			if ((a->rcfg += 1) >= 3) {
				pop_cfg();
				a->rcfg = 0;
			}
		}
		break;
	case 3: at_send("CTRL=,5,0");
		a->scfg = 4;
		a->tout = gl.tnow + S2TIC(0.50);
		break;
	case 4:	if (!(gl.navh.cst & AS_COMMAND_ACK)) a->scfg = 1;
		else if ((int)(gl.tnow - a->tout) >= 0) a->scfg = 3;
		break;
	case 5: if (gl.navh.cst & AS_COMMAND_ACK) a->scfg = 3;
		else if ((int)(gl.tnow - a->tout) >= 0) {
			a->scfg = 1;
			gl.dat.typ = gl.dat.nbu = 0;
			if ((a->rcfg += 1) >= 3) a->cdrq = a->rcfg = 0;
		}
		break;
	}

	/* data for Arduino */
	if (gl.tnow >= a->sdto && a->link > 0) {
		a->sdto = gl.tnow + S2TIC(0.50);
		m2a.dcst = gl.navh.cst;
		m2a.cbat = gl.nt00.bat;
		m2a.sgps = 0;
		d = cmd;
		*d++ = '$';
		d = addhexp(&m2a,sizeof(m2a),d);
		*d++ = '*';
		*d++ = '\n';
		write(gl.ad,cmd,d-cmd);
	}
}

static void rxcmd(int na, char **av)
{
	char	c, *s, *d = 0, cmd[256];
	vif_t	v[4];
	int	i, k;

	if (na < 2) return;
	if (av[0][1] != 'X') return;
	if (av[0][2]) return;

	switch ((c = av[1][0])) {
	case 'V':
		if (na < 3 || av[2][1]) break;
		switch ((c = av[2][0])) {
		case 'Z': /* video ZAP */
			gl.vzap++; gl.vzap &= 03;
			sprintf(cmd,"%d",gl.vzap);
			psh_cfg("video:video_channel",cmd);
			break;
		case '0': /* stop  video recording */
		case '1': /* start video recording */
			if (!gl.ard2) break;
			if (c == '1') {
				if (!gl.rvid) break;
				if (gl.usbc < (5*1024)) break;
				if (gl.usbf < (5*1024)) break;
				psh_cfg("video:video_codec","130");
				psh_cfg("userbox:userbox_cmd","1");
			}
			else {
				psh_cfg("video:video_codec","129");
				psh_cfg("userbox:userbox_cmd","0");
			}
			break;
#if	0
		case '2': /* snapshot */
			psh_cfg("userbox:userbox_cmd","2,0,0,2013_000");
			break;
#endif
		}
		break;
	case 'L': /* blink leds */
		if (na < 5) break;
		sprintf(d=cmd,"LED=,%s,%s,%s",av[2],av[3],av[4]);
		break;
	case 'C': /* summary config */
		if (gl.ardu.acfg || na < 2) break;
		for (i = 2; i < na; i++) {
			s = 0;
			switch (i) {
			case 2: s = "control:outdoor"; break;
			case 3: s = "control:flight_without_shell"; break;
			case 4: s = "control:euler_angle_max"; break;
			case 5: s = "control:control_vz_max"; break;
			case 6: s = "control:control_yaw"; break;
			case 7: s = "control:altitude_max";
				k = strtol(av[i],0,0);
				if (k <=    0) k = gl.ard2 ? 1000 : 10;
				if (k <= 1000) k *= 1000;
				sprintf(av[i]=cmd,"%d",k);
				break;
			case 8: gl.anim = strtol(av[i],0,0);
				break;
			case 9: if (!gl.ard2) break;
				s = "video:video_on_usb";
				gl.rvid = strtol(av[i],0,0);
				break;
			}
			if (s && av[i] && *av[i]) psh_cfg(s,av[i]);
		}
		gl.ardu.acfg = 1;
		break;
	case 'T': /* trimm */
		d = cmd + sprintf(cmd,"FTRIM=\r"LED_BLINK_GREEN",1");
		SLOG("TRIM");
		break;
	case 'A': /* animation trigger */
		if (na < 3) break;
		if ((i = av[2][0]-'1') >= 0 && i < 4) {
			/* flip/nod animations
			 *     drone 2         drone 1
			 * S 0 17,15 bck flip  3,1000 bck nod
			 * W 1 18,15 lft flip  0,1000 lft nod
			 * N 2 16,15 fwd flip  2,1000 fwd nod
			 * E 3 19,15 rgt flip  1,1000 rgt nod
			 */
			if (gl.ard2) i += 4;
			sprintf(cmd,"%d,%d","\x03\x00\x02\x01\x11\x12\x10\x13"[i],gl.ard2?15:1000);
			psh_cfg("control:flight_anim",cmd);
		}
		break;
	case '0': /* LAND */
	case '1': /* FM_1 */
	case '2': /* FM_2 */
	case '3': /* FM_3 */
	case 'E': /* emergency */
		if (gl.ardu.usr.mode != c) SLOG("FMOD,%c",c);
		gl.ardu.usr.mode = c;
		bzero(gl.ardu.usr.rpgy,sizeof(gl.ardu.usr.rpgy));
		if (na < 6) break;
		for (i = 0; i < 4; i++) {
			v[i].i = strtol(av[2+i],&s,10);
			if (*s) break;
		}
		if (i < 4) break;
		for (i = 0; i < 4; i++)
			if (v[i].i) gl.ardu.usr.rpgy[i].f = (float)v[i].i/1000.0;
		break;
	}
	if (d) at_send(cmd);
}

static void idv_in(void)
{
	int	nk, na, psh;
	char	*s, *e, *a, *b, *av[8], dat[1028], tmp[8];

	if ((nk = read(gl.cd,dat,sizeof(dat)-4)) <= 1) return;
	if (gl.idev.trcv == 0) SLOG("IDEV,1");
	gl.idev.trcv = gl.tnow;
	for (e = (b = dat) + nk; b < e; b++) {
		while (b < e && *b <= ' ') b++;
		for (s = b; b < e && *b >= ' '; b++); *b = 0;
		if ((b - s) < 4) continue;
		if (!strbeg("AT*C",s)) continue;
		if ((na = cmd2av(s+3,av,NEL(av))) != 4) {
			if (na > 0 && strequ(av[0],"COMWDG")) {
				gl.idev.fcst &= ~AS_COM_WATCHDOG;
				continue;
			}
			continue;
		}
		if (strequ(av[0],"CTRL")) {
			/* CTRL <seq> <typ> 0
			 * 0    1     2     3  4 */
			switch (av[2][0]) {
			case '4': /* asking for config dump */
			case '6': /* asking for config choices */
				if (gl.idev.fcst & AS_COMMAND_ACK) {
					SLOG("busy !");
					break; /* is busy as far as iDev is concerned */
				}
				if (gl.ardu.cdrq) break;
				gl.idev.fcst |=  AS_COMMAND_ACK;
				gl.ardu.cdrq = av[2][0] - '0';
				break;
			case '5': /* ack AS_COMMAND_ACK */
				gl.idev.fcst &= ~AS_COMMAND_ACK;
				break;
			}
			continue;
		}
		if (strequ(av[0],"CONFIG")) {
			/* CONFIG <seq> <var> <val>
			 * 0      1     2     3     4 */
			if (gl.idev.fcst & AS_COMMAND_ACK) {
				SLOG("busy !");
				continue; /* is busy as far as iDev is concerned */
			}
			gl.idev.fcst |= AS_COMMAND_ACK;
			if (strbeg("custom:",av[2])) {
				a = &av[2][7];
				if (strequ(a,"session_id")) {
					if (strequ(av[3],"-all")) {
						gl.idev.sid[0] = 0;
						gl.idev.sds[0] = 0;
						gl.idev.aid[0] = 0;
						gl.idev.pid[0] = 0;
					}
					else strtxf(av[3],gl.idev.sid,sizeof(gl.idev.sid)-1);
				}
				else if (strequ(a,"application_id"))
					strtxf(av[3],gl.idev.aid,sizeof(gl.idev.aid)-1);
				else if (strequ(a,"profile_id"))
					strtxf(av[3],gl.idev.pid,sizeof(gl.idev.pid)-1);
				else if (strequ(a,"session_desc"))
					strtxf(av[3],gl.idev.sds,sizeof(gl.idev.sds)-1);
				continue;
			}
			psh = 0;
			if (strbeg("video:",av[2])) {
				if (strbeg("video_chan",&av[2][6])) {
					gl.vzap++; gl.vzap &= 03;
					sprintf(av[3]=tmp,"%d",gl.vzap);
					psh++;
				}
				else if (!gl.rvid) psh++;
			}
			else if (strbeg("userbox:",av[2])) {
				if (!gl.rvid) psh++;
			}
			else if (strbeg("gps:",av[2])) psh++;
			else if (strbeg("general:",av[2])) {
				if (strbeg("navdata_",&av[2][8])) {
					if (av[2][16] == 'd') gl.idev.nall = 1;
					else psh++;
				}
				else if (strbeg("local",&av[2][8])) psh++;
			}
			if (psh) psh_cfg(av[2],av[3]);
			continue;
		}
	}
}

/* iDev data port connect gets redirected here */
static void idd_in(void)
{
	int	fd, one = 1;
	soa_t	fa;
	socklen_t fl;

	fl = sizeof(fa.sa);
	if ((fd = accept(gl.id,&fa.sa,&fl)) <= 0) return;
	if (gl.fd > 0 || ioctl(fd,FIONBIO,&one)) close(fd);
	else add_fd(gl.fd=fd);
}

/* iDev data port */
static void fds_in(void)
{
	int	n;
	u08_t	dat[1024];

	if ((n = read(gl.fd,dat,sizeof(dat))) > 0) return;
	del_fd(gl.fd);
	close(gl.fd);
	gl.fd = -1;
}

/*
 * rx2at input
 */
static void ard_in(void)
{
	int	n;
	char	*s, *e, *b, *av[32], dat[512];

	if ((n = read(gl.ad,dat,sizeof(dat))) <= 1) return;
	if (--n == 0) return; /* strip terminating '\n' */
	if (gl.ardu.trcv == 0) gl.ardu.link = 1, SLOG("LINK,1");
	else if (gl.ardu.link < 0) return;
	gl.ardu.trcv = gl.tnow;
	for (e = (s = dat) + n; s < e; s++) {
		while (s < e && *s <= ' ') s++;
		for (b = s; s < e && *s >= ' '; s++);
		if ((s - b) < 4) continue;
		*s = 0;
		     if (b[0] == 'R') rxcmd(cmd2av(b,av,NEL(av)),av);
		else if (b[0] == '$') gps_in(b);
		else if (b[0] == 'B') bmp_in(b);
		else if (b[0] == 'H') hmc_in(b);
		else if (strbeg("slog,",b) && (s-b) > 5) SLOG("%s",b+5);
	}
}

/* drone data port sends here */
static void dat_in(void)
{
	char	*s, *d, *b, *e, *t;
	int	n, k;

	k = sizeof(gl.dat.buf) - gl.dat.nbu;
	if (k <= 0) gl.dat.typ = gl.dat.nbu = 0, k = sizeof(gl.dat.buf);
	if ((n = read(gl.dd,gl.dat.buf+gl.dat.nbu,k)) < 0) {
		gl.dat.typ = gl.dat.nbu = 0;
		return;
	}
	SLOG("DATI,%d,%d",gl.dat.nbu,n);
	if (n == 0) {
		/* shutdown connection to drone data port */
		del_fd(gl.dd);
		close(gl.dd);
		gl.dd = -1;
		return;
	}
	gl.dat.nbu += n;
	if (gl.dat.buf[gl.dat.nbu-1]) return; /* there is more ... */
	SLOG("DATI,EOF,%d",gl.dat.typ);
	e = (d = gl.dat.buf) + gl.dat.nbu - 1;
	switch (gl.dat.typ) {
	default: gl.dat.nbu = 0; break;
	case 4: /* answer to CTRL=,4s ... config parameters */
		/* take out 'custom:.*' */
		t = "custom:";
		for (s = d; s < e; ) {
			for (b = s; s < e && *s >= ' '; s++);
			while (s < e && *s < ' ') s++;
			if (*b == *t && strbeg(t,b)) continue;
			if (d == b) d = s;
			else while (b < s) *d++ = *b++;
		}
		/* add custom:... iDev wants to see */
		d += sprintf(d,
			"%sapplication_id = %s\n"
			"%sprofile_id = %s\n"
			"%ssession_id = %s\n"
			"%ssession_desc = %s\n",
			t,gl.idev.aid,t,gl.idev.pid,
			t,gl.idev.sid,t,gl.idev.sds);
		gl.dat.nbu = ++d - gl.dat.buf;
		break;
	case 6: /* answer to CTRL=,6 */
		/* take out session section */
		for (n = 0, s = d; s < e; ) {
			for (b = s; s < e && *s >= ' '; s++);
			while (s < e && *s < ' ') s++;
			if (*b == '[') n = strbeg("sess",b+1) ? 0 : 1;
			if (n == 0) continue;
			if (d == b) d = s;
			else while (b < s) *d++ = *b++;
		}
		/* add session section iDev wants to see */
		d += sprintf(d,"[sessions]\n");
		if (gl.idev.sid[0]) d += sprintf(d,"%s,%s\n",gl.idev.sid,gl.idev.sds);
		gl.dat.nbu = ++d - gl.dat.buf;
		break;
	}
	if (gl.fd > 0 && gl.dat.nbu > 0) {
		n = write(gl.fd,gl.dat.buf,gl.dat.nbu);
		SLOG("DATI,SND,%d,%d",gl.dat.nbu,n);
	}
	gl.ardu.cdrq = gl.dat.typ = gl.dat.nbu = 0;
}

static void grab_ctrl(void)
{
	at_send(LED_BLINK_ORANGE",1");	/* show user we are here */
	at_send("PMODE=,2\rMISC=,2,20,2000,3000"); /* FreeFlight does these... */
	if (gl.idse) {
		psh_cfg("custom:session_id","-all");
		psh_cfg("custom:session_id",gl.sid);
		psh_cfg("custom:application_id",gl.aid);
		psh_cfg("custom:profile_id",gl.pid);
		psh_cfg("custom:session_desc",gl.sds);
		if (gl.ard2) {
			psh_cfg("video:codec_fps","25");
			psh_cfg("video:video_codec","129");
			psh_cfg("video:max_bitrate","1500");
		}
		else	psh_cfg("video:video_codec","64");
	}
	/* default configuration */
	psh_cfg("detect:detect_type","3");	/* no detections */
	psh_cfg("control:control_level","0");
	psh_cfg("general:navdata_demo","1");
	if (gl.ard2) psh_cfg("general:navdata_options","0x6500001"); /* tags 26, 25, 22, 20, 0 */
}

static void sleep_ms(int nms)
{
	tmv_t	tv;
	int	n;

	tv.tv_sec = 0;
	for (n = 0; n < nms; n += 10) {
		tv.tv_usec = 10000;
		select(0,0,0,0,&tv);
	}
}

static void ipt_clr(void)
{
	system("iptables -P INPUT ACCEPT ; iptables -F ; iptables -t nat -F");
}

static int ipt_natrri(int prot, int port, int palt)
{
	char	buf[256];

	sprintf(buf,"iptables -t nat -A PREROUTING -p %s ! -s 127.0.0.1 --dport %d -j DNAT --to :%d",
		prot==IPPROTO_TCP?"tcp":"udp",ntohs(port),ntohs(palt));
	return system(buf);
}

static int udp5556_traffic(void)
{
	int	fd, n = 0;
	char	lpf[16], fnm[32], buf[1024];

	sprintf(lpf,"CK5556_%d",getpid());
	sprintf(buf,"iptables -A INPUT -p udp --dport 5556 -j LOG --log-prefix \"%s \"",lpf);
	system(buf);
	sleep_ms(100);
	buf[10] = 'D';
	system(buf);

	sprintf(fnm,"/tmp/%s.udp",lpf);
	sprintf(buf,"dmesg | grep \"%s \" | tail -1 > %s",lpf,fnm);
	system(buf);
	if ((fd = open(fnm,0)) > 0) {
		n = read(fd,buf,sizeof(buf)-4);
		close(fd);
	}
	unlink(fnm);
	return (fd > 0 && n == 0) ? 0 : -1;
}

static void usr_cmd(void)
{
	vif_t	*v = gl.ardu.usr.rpgy;
	char	*d = 0, c, cmd[128];

	c = gl.ardu.link <= 0 ? '0' : gl.ardu.usr.mode;
	switch (c) {
	case '1':
	case '2':
	case '3':
		d = cmd;
		d += sprintf(d,"PCMD%s=,0",gl.ard2?"_MAG":"");
		switch (c) {
		case '1': if (v[0].i || v[1].i) d[-1] = '1'; break;
		case '2': d[-1] = '1'; break;
		case '3': d[-1] = '1'; break;
		}
		d += sprintf(d,",%d,%d,%d,%d%s",v[0].i,v[1].i,v[2].i,v[3].i,gl.ard2?",0,0":"");
		SLOG(cmd);
		break;
	}
	if (d) *d++ = '\r'; else d = cmd;
	gl.ref = REF_LAND;
	switch (c) {
	case '1':
	case '2':
	case '3': gl.ref = REF_STRT; break;
	case 'E': gl.ref = REF_ESTP; break;
	}
	sprintf(d,"REF=,%u",gl.ref);
	at_send(cmd);
}

static void loop(void)
{
	fd_set	rd;
	tmv_t	tv;
	int	n, u, s;

	connect(gl.dd,&gl.ddat.sa,sizeof(gl.ddat.sa));
	gl.tnow = tnow();
	grab_ctrl();
	n = 1, sendto(gl.rd,&n,sizeof(n),0,&gl.dnav.sa,sizeof(gl.dnav.sa));
	tv.tv_sec = 0;
	SLOG("LOOP,1");
	u = 0;
	while (!gl.exit) {
		if (gl.idev.trcv && (tnow() - gl.idev.trcv) >= S2TIC(0.5)) {
			SLOG("IDEV,0");
			gl.idev.fcst |= AS_COM_WATCHDOG;
			gl.idev.trcv = 0;
		}
		ard_sq();
		if (u) u--, usr_cmd();
		if ((n = gl.nat) > 0) sendto(gl.cd,gl.atb,n,0,&gl.dcmd.sa,sizeof(gl.dcmd.sa));
		gl.nat = 0;
		rd = gl.sd;
		tv.tv_usec = 100000;
		n = select(gl.md,&rd,0,0,&tv);
		if (n < 0 && errno != EINTR) break;
		gl.tnow = tnow();
		if (n > 0 && (s = gl.ad) > 0 && FD_ISSET(s,&rd)) n--, ard_in();
		if (n > 0 && (s = gl.cd) > 0 && FD_ISSET(s,&rd)) n--, idv_in();
		if (n > 0 && (s = gl.rd) > 0 && FD_ISSET(s,&rd)) n--, nav_in(), u++;
		if (n > 0 && (s = gl.nd) > 0 && FD_ISSET(s,&rd)) n--, nrq_in();
		if (n > 0 && (s = gl.dd) > 0 && FD_ISSET(s,&rd)) n--, dat_in();
		if (n > 0 && (s = gl.id) > 0 && FD_ISSET(s,&rd)) n--, idd_in();
		if (n > 0 && (s = gl.fd) > 0 && FD_ISSET(s,&rd)) n--, fds_in();
	}
	SLOG("LOOP,0");
}

void cleanup(void)
{
	int	s;

	SLOG("EXIT,\"%s\"",version+5);
	ipt_clr();
	if ((s = gl.cd) > 0) gl.cd = -1, close(s);
	if ((s = gl.nd) > 0) gl.nd = -1, close(s);
	if ((s = gl.rd) > 0) gl.rd = -1, close(s);
	if ((s = gl.dd) > 0) gl.dd = -1, close(s);
	if ((s = gl.id) > 0) gl.id = -1, close(s);
	if ((s = gl.fd) > 0) gl.fd = -1, close(s);
	if ((s = gl.ad) > 0) {
		gl.ad = -1;
		write(s,"\004",1);
		sleep_ms(10);
		tcsetattr(s,TCSADRAIN,gl.ap);
		close(s);
	}
	if ((s = gl.ld) > 0) gl.ld = -1, close(s);
}

static int mk_soc(int typ, soa_t *sa)
{
	int	sd, n;
	socklen_t sl;

	sa->sin_fmly = AF_INET;
	if ((sd = socket(sa->sin_fmly,typ,0)) < 0) return -1;
	while (1) {
		n = 1; if (ioctl(sd,FIONBIO,&n)) break;
		sl = sizeof(sa->sa);
		if (bind(sd,&sa->sa,sl)) break;
		if (getsockname(sd,&sa->sa,&sl)) break;
		add_fd(sd);
		return sd;
	}
	n = errno; close(sd); errno = n;
	return -1;
}

int main(int ac, char **av, char **env)
{
	char	*p;
	int	s;

	bzero(&gl,sizeof(gl));
	FD_ZERO(&gl.sd);
	gl.sid = "00000001"; /* some number != 0 will do */
	gl.pid = "6fb0c592"; /* crc32 of 'mirumod' */
	gl.aid = "e98c4390"; /* crc32 of 'mirumod:0.20' */
	gl.sds = "mirumod";
	if (tref()) return errno;
	for (s = 0; ++s < ac; )
		if (*(p = av[s]) == '-')
			while (*(++p)) switch (*p) {
			case 'w': gl.flag |= F_BLK_WIFI; break;
			case 'l': if (++s < ac) gl.lgid = strtol(av[s],0,0); break;
			}
	if (EN_SLOG) {
		if (gl.lfn[0] == 0) strcpy(gl.lfn,"/tmp/at2so.log");
		gl.ld = open(gl.lfn,O_CREAT|O_TRUNC|O_WRONLY,0664);
	}
	SLOG("BOOT,\"%s\",%d",version+5,gl.lgid);

	if ((s = open("/update/version.txt",0)) > 0) {
		read(s,gl.dfmw,sizeof(gl.dfmw)-1);
		close(s);
		for (p = gl.dfmw, s = NEL(gl.dfmw); --s >= 0 && *p; p++)
			if (*p == '\r' || *p == '\n') *p = 0;
		SLOG("DFMW,%s",gl.dfmw);
		if ((s = strtol(gl.dfmw,&p,10)) >= 2) gl.ard2 = gl.idse = 1;
		else if (s == 1 && *p == '.' && strtol(p+1,0,10) >= 7) gl.idse = 1;
	}

	gl.dcmd.sin_fmly = AF_INET; gl.dcmd.sin_addr = IPV4A(127,0,0,1); /* localhost */
	gl.dnav = gl.dcmd;
	gl.ddat = gl.dcmd;
	gl.dnav.sin_port = htons(5554);
	gl.dcmd.sin_port = htons(5556);
	gl.ddat.sin_port = htons(5559);

	/* serial receiver (Arduino)
	 * using ICANON without ICRNL, -> read/select complete when '\n' is received */
	if ((gl.ad = open("/dev/tty",2)) < 0) return errno;
	if (tcgetattr(gl.ad,gl.ap)) return errno;
	gl.ap[1] = gl.ap[0];
	gl.ap[1].c_iflag = IGNBRK|IGNPAR|ISTRIP;
	gl.ap[1].c_lflag = ICANON; 	
	if (tcsetattr(gl.ad,TCSANOW,&gl.ap[1])) return errno;
	add_fd(gl.ad);

	atexit(cleanup);

	if (udp5556_traffic()) return -1;

	/* sockets */
	if ((gl.rd = mk_soc(SOCK_DGRAM ,&gl.ra)) < 0) return errno; /* int: sink for navdata from drone */
	if ((gl.nd = mk_soc(SOCK_DGRAM ,&gl.na)) < 0) return errno; /* ext: navdata  */
	if ((gl.cd = mk_soc(SOCK_DGRAM ,&gl.ca)) < 0) return errno; /* int: src for drone commands */
	if ((gl.dd = mk_soc(SOCK_STREAM,&gl.da)) < 0) return errno; /* int: sink for drone data */
	if ((gl.id = mk_soc(SOCK_STREAM,&gl.ia)) < 0) return errno; /* ext: data/ctrl port */
	listen(gl.id,1);

	ipt_clr();
	if (gl.flag & F_BLK_WIFI) system("iptables -A INPUT -i ath0 -j DROP");
	else {
		system("iptables -A INPUT -p tcp --dport 5557 -j DROP");
		if (ipt_natrri(IPPROTO_UDP,gl.dcmd.sin_port,gl.ca.sin_port)) return errno;
		if (ipt_natrri(IPPROTO_UDP,gl.dnav.sin_port,gl.na.sin_port)) return errno;
		if (ipt_natrri(IPPROTO_TCP,gl.ddat.sin_port,gl.ia.sin_port)) return errno;
	}

	loop();

	return errno;
}

