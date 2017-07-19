/* A Bison parser, made by GNU Bison 3.0.4.  */

/* Bison implementation for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "3.0.4"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1




/* Copy the first part of user declarations.  */
#line 2 "lcp.y" /* yacc.c:339  */

#include "reader.h"

int yyerror(char *s);
int yylex(void);
int negation(Formula inf, Formula outf, Formula& negform);
int totaltreeleaves(Formula *node);
void negation_movein(Formula* f);
void addtree(Formula *f, Formula *node, int totalrank, int i);
void copy_formula(Formula *f, Formula *cp);
int ranktree(Formula *node, int rank, int pos);
int derivedname=0; 
string strupcase(const string& str );


extern Reader reader;
typedef map<string,string>::value_type value_type;
map<string, string> t_map;


#line 87 "lcp.tab.c" /* yacc.c:339  */

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* In a future release of Bison, this section will be replaced
   by #include "lcp.tab.h".  */
#ifndef YY_YY_LCP_TAB_H_INCLUDED
# define YY_YY_LCP_TAB_H_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int yydebug;
#endif

/* Token type.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    dDOMAIN = 258,
    DEFINE = 259,
    REQUIREMENTS = 260,
    STRIPS = 261,
    ADL = 262,
    TYPING = 263,
    NEGATIVE_PRECONDITIONS = 264,
    DISJUNCTIVE_PRECONDITIONS = 265,
    EQUALITY = 266,
    EXISTENTIAL_PRECONDITIONS = 267,
    UNIVERSAL_PRECONDITIONS = 268,
    QUANTIFIED_PRECONDITIONS = 269,
    CONDITIONAL_EFFECTS = 270,
    DERIVED_PREDICATES = 271,
    TYPES = 272,
    CONSTANTS = 273,
    PREFERENCE = 274,
    PREDICATES = 275,
    CONSTRAINTS = 276,
    STRUCTURE = 277,
    PROBLEM = 278,
    INIT = 279,
    GOAL = 280,
    OBJECT = 281,
    pDOMAIN = 282,
    ACTION = 283,
    DERIVED = 284,
    PRECONDITION = 285,
    EFFECT = 286,
    PARAMETERS = 287,
    WHEN = 288,
    AND = 289,
    OR = 290,
    NOT = 291,
    IMPLY = 292,
    EXISTS = 293,
    FORALL = 294,
    ATEND = 295,
    ALWAYS = 296,
    SOMETIME = 297,
    WITHIN = 298,
    AT_MOST_ONCE = 299,
    SOMETIME_AFTER = 300,
    SOMETIME_BEFORE = 301,
    ALWAYS_WITHIN = 302,
    HOLD_DURING = 303,
    HOLD_AFTER = 304,
    EITHER = 305,
    UNKNOWN = 306,
    ONEOF = 307,
    NUMBER = 308,
    NAME = 309,
    VARIABLE = 310,
    MINUS = 311,
    EQUAL = 312,
    LEFT_PAREN = 313,
    RIGHT_PAREN = 314,
    SEMICOLON = 315
  };
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED

union YYSTYPE
{
#line 23 "lcp.y" /* yacc.c:355  */

  string*	str_val;
  StringList*  str_list; 
  StringList2* str_list2;
  
  Term* term_;
  TermList* termlist_;
  Predicate* predicate_;
  Atom* atom_;
  AtomList* atomlist_;
  LiteralTerm* literal_;
  LiteralList* literallist_;
  Formula* formula_; 
  Type* type_;
  TypeList* typelist_;
  Requirement* requirement_;
  Structure* structure_;
  StructureList* structurelist_;
  pDomain* domain_;
  pProblem* problem_;

#line 210 "lcp.tab.c" /* yacc.c:355  */
};

typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE yylval;

int yyparse (void);

#endif /* !YY_YY_LCP_TAB_H_INCLUDED  */

/* Copy the second part of user declarations.  */

#line 227 "lcp.tab.c" /* yacc.c:358  */

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#else
typedef signed char yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(Msgid) dgettext ("bison-runtime", Msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(Msgid) Msgid
# endif
#endif

#ifndef YY_ATTRIBUTE
# if (defined __GNUC__                                               \
      && (2 < __GNUC__ || (__GNUC__ == 2 && 96 <= __GNUC_MINOR__)))  \
     || defined __SUNPRO_C && 0x5110 <= __SUNPRO_C
#  define YY_ATTRIBUTE(Spec) __attribute__(Spec)
# else
#  define YY_ATTRIBUTE(Spec) /* empty */
# endif
#endif

#ifndef YY_ATTRIBUTE_PURE
# define YY_ATTRIBUTE_PURE   YY_ATTRIBUTE ((__pure__))
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# define YY_ATTRIBUTE_UNUSED YY_ATTRIBUTE ((__unused__))
#endif

#if !defined _Noreturn \
     && (!defined __STDC_VERSION__ || __STDC_VERSION__ < 201112)
# if defined _MSC_VER && 1200 <= _MSC_VER
#  define _Noreturn __declspec (noreturn)
# else
#  define _Noreturn YY_ATTRIBUTE ((__noreturn__))
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(E) ((void) (E))
#else
# define YYUSE(E) /* empty */
#endif

#if defined __GNUC__ && 407 <= __GNUC__ * 100 + __GNUC_MINOR__
/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN \
    _Pragma ("GCC diagnostic push") \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")\
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# define YY_IGNORE_MAYBE_UNINITIALIZED_END \
    _Pragma ("GCC diagnostic pop")
#else
# define YY_INITIAL_VALUE(Value) Value
#endif
#ifndef YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_END
#endif
#ifndef YY_INITIAL_VALUE
# define YY_INITIAL_VALUE(Value) /* Nothing. */
#endif


#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined EXIT_SUCCESS
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
      /* Use EXIT_SUCCESS as a witness for stdlib.h.  */
#     ifndef EXIT_SUCCESS
#      define EXIT_SUCCESS 0
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's 'empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined EXIT_SUCCESS \
       && ! ((defined YYMALLOC || defined malloc) \
             && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef EXIT_SUCCESS
#    define EXIT_SUCCESS 0
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined EXIT_SUCCESS
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined EXIT_SUCCESS
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
         || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

# define YYCOPY_NEEDED 1

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)                           \
    do                                                                  \
      {                                                                 \
        YYSIZE_T yynewbytes;                                            \
        YYCOPY (&yyptr->Stack_alloc, Stack, yysize);                    \
        Stack = &yyptr->Stack_alloc;                                    \
        yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
        yyptr += yynewbytes / sizeof (*yyptr);                          \
      }                                                                 \
    while (0)

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from SRC to DST.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(Dst, Src, Count) \
      __builtin_memcpy (Dst, Src, (Count) * sizeof (*(Src)))
#  else
#   define YYCOPY(Dst, Src, Count)              \
      do                                        \
        {                                       \
          YYSIZE_T yyi;                         \
          for (yyi = 0; yyi < (Count); yyi++)   \
            (Dst)[yyi] = (Src)[yyi];            \
        }                                       \
      while (0)
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  4
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   279

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  61
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  48
/* YYNRULES -- Number of rules.  */
#define YYNRULES  109
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  239

/* YYTRANSLATE[YYX] -- Symbol number corresponding to YYX as returned
   by yylex, with out-of-bounds checking.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   315

#define YYTRANSLATE(YYX)                                                \
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, without out-of-bounds checking.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60
};

#if YYDEBUG
  /* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   188,   188,   198,   202,   209,   215,   222,   228,   234,
     242,   252,   264,   269,   275,   283,   290,   297,   304,   309,
     364,   373,   384,   388,   395,   402,   413,   419,   427,   433,
     441,   446,   453,   457,   467,   476,   484,   491,   503,   518,
     522,   532,   537,   547,   569,   574,   581,   587,   592,   603,
     607,   617,   621,   632,   639,   645,   653,   663,   667,   695,
     743,   748,   755,   763,   773,   784,   791,   807,   811,   825,
     845,   849,   861,   881,   888,   893,   918,   925,   932,   939,
     946,   953,   960,   968,   976,   986,   998,  1005,  1010,  1015,
    1023,  1037,  1046,  1070,  1097,  1130,  1134,  1145,  1149,  1174,
    1181,  1190,  1200,  1206,  1214,  1218,  1226,  1235,  1246,  1259
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || 0
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "dDOMAIN", "DEFINE", "REQUIREMENTS",
  "STRIPS", "ADL", "TYPING", "NEGATIVE_PRECONDITIONS",
  "DISJUNCTIVE_PRECONDITIONS", "EQUALITY", "EXISTENTIAL_PRECONDITIONS",
  "UNIVERSAL_PRECONDITIONS", "QUANTIFIED_PRECONDITIONS",
  "CONDITIONAL_EFFECTS", "DERIVED_PREDICATES", "TYPES", "CONSTANTS",
  "PREFERENCE", "PREDICATES", "CONSTRAINTS", "STRUCTURE", "PROBLEM",
  "INIT", "GOAL", "OBJECT", "pDOMAIN", "ACTION", "DERIVED", "PRECONDITION",
  "EFFECT", "PARAMETERS", "WHEN", "AND", "OR", "NOT", "IMPLY", "EXISTS",
  "FORALL", "ATEND", "ALWAYS", "SOMETIME", "WITHIN", "AT_MOST_ONCE",
  "SOMETIME_AFTER", "SOMETIME_BEFORE", "ALWAYS_WITHIN", "HOLD_DURING",
  "HOLD_AFTER", "EITHER", "UNKNOWN", "ONEOF", "NUMBER", "NAME", "VARIABLE",
  "MINUS", "EQUAL", "LEFT_PAREN", "RIGHT_PAREN", "SEMICOLON", "$accept",
  "pdomain", "domain_option", "pproblem", "problem_option", "object_dec",
  "init", "goal", "init_list", "action_def", "action_def_body",
  "precondition_def", "effect_def", "effectlist", "c_effect_star_and",
  "c_effect", "p_effect", "p_effect_star_and", "cond_effect",
  "derived_def", "pref_GD", "pre_GD", "pre_GD_star_and", "pre_GD_star_or",
  "predicates_def", "atomic_formula_skeleton_plus",
  "atomic_formula_skeleton", "typed_list_variable", "type",
  "primitive_type_plus", "primitive_type", "constants_def", "types_def",
  "typed_list_name", "typed_list_object", "require_def",
  "require_key_plus", "require_key", "GD", "GD_star_or", "GD_star_and",
  "literal", "atomic_formula_term", "predicate", "term_star", "term",
  "name", "variable", YY_NULLPTR
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[NUM] -- (External) token number corresponding to the
   (internal) symbol number NUM (which must be that of a token).  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301,   302,   303,   304,
     305,   306,   307,   308,   309,   310,   311,   312,   313,   314,
     315
};
# endif

#define YYPACT_NINF -140

#define yypact_value_is_default(Yystate) \
  (!!((Yystate) == (-140)))

#define YYTABLE_NINF -1

#define yytable_value_is_error(Yytable_value) \
  0

  /* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
     STATE-NUM.  */
static const yytype_int16 yypact[] =
{
     -49,    22,    51,   -39,  -140,    52,     6,  -140,    11,  -140,
      19,   110,    17,  -140,  -140,  -140,  -140,  -140,  -140,   220,
       6,     6,    26,     6,    32,    87,  -140,  -140,  -140,  -140,
    -140,  -140,  -140,  -140,  -140,  -140,    97,  -140,    41,   -24,
      55,     5,    -5,    39,  -140,    79,  -140,    58,    66,    71,
    -140,  -140,  -140,     6,  -140,  -140,   -46,  -140,  -140,    32,
    -140,  -140,  -140,    78,   103,   109,    65,   114,  -140,    36,
     129,  -140,   -46,  -140,   169,     6,   147,     6,  -140,  -140,
     135,   134,  -140,  -140,  -140,  -140,  -140,   -23,  -140,  -140,
    -140,  -140,    32,  -140,  -140,   141,  -140,   168,   150,    32,
       6,  -140,     6,  -140,  -140,     6,  -140,  -140,   129,   129,
     142,   152,  -140,  -140,  -140,   129,  -140,   141,   153,  -140,
     154,    -5,   155,   -11,  -140,   157,  -140,  -140,   158,    -1,
    -140,   129,    76,    82,    84,   159,   155,   129,    32,    32,
      89,    91,   163,    93,   164,    32,   124,  -140,  -140,  -140,
    -140,  -140,    32,   166,  -140,  -140,   173,   144,  -140,  -140,
    -140,  -140,  -140,  -140,  -140,   174,   175,   178,  -140,  -140,
     -29,  -140,   179,    29,  -140,  -140,  -140,   180,   181,   183,
     185,  -140,  -140,   129,   186,   187,  -140,   188,  -140,   129,
       6,   184,   189,   190,    38,  -140,   -12,   191,   190,   192,
    -140,  -140,  -140,    99,  -140,   194,  -140,  -140,    15,   195,
    -140,   196,   197,   198,     6,   223,   196,   232,   199,   233,
    -140,   125,   201,   202,   203,   186,  -140,  -140,  -140,  -140,
    -140,  -140,   204,   106,   118,  -140,  -140,  -140,  -140
};

  /* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
     Performed when YYTABLE does not specify something else to do.  Zero
     means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       0,     0,     0,     0,     1,     0,     0,   108,     0,     3,
       0,     0,     0,     9,     8,     7,     6,     5,     4,     0,
      67,    70,     0,     0,    57,     0,     2,    76,    77,    79,
      81,    80,    78,    82,    83,    84,     0,    74,     0,    67,
       0,    70,     0,     0,    54,    22,   109,     0,    57,     0,
      73,    75,    66,     0,    69,    65,     0,    72,   103,    57,
     102,    53,    55,     0,     0,     0,     0,    23,    24,     0,
       0,    99,     0,    59,     0,    67,     0,    70,    61,    64,
       0,     0,    46,    27,    45,    86,    85,     0,    29,    30,
      36,    37,    57,    21,    25,     0,   104,     0,     0,    57,
       0,    68,     0,    71,    56,     0,    49,    95,     0,     0,
       0,     0,    95,    95,    26,     0,    32,     0,     0,    28,
       0,     0,     0,     0,    97,     0,    43,    58,     0,     0,
      62,     0,     0,     0,     0,     0,     0,     0,    57,    57,
       0,     0,     0,     0,     0,    57,    22,   100,   101,   105,
     106,   107,    57,     0,    60,    63,     0,     0,    47,    50,
      87,    98,    88,    96,    89,     0,     0,     0,    93,    94,
       0,    41,     0,     0,    31,    33,    38,     0,     0,     0,
       0,    44,    90,     0,     0,     0,    35,     0,    20,     0,
       0,     0,     0,     0,     0,    39,     0,     0,     0,     0,
      92,    48,    91,     0,    34,     0,    42,    40,     0,     0,
      13,     0,     0,     0,    70,     0,     0,     0,     0,     0,
      14,     0,     0,     0,     0,     0,    11,    49,    51,    16,
      15,    10,     0,     0,     0,    17,    18,    19,    52
};

  /* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -140,  -140,  -140,  -140,  -140,     4,    56,    48,  -140,  -140,
     120,  -140,   200,    81,  -140,   126,  -139,  -140,  -140,  -140,
    -140,   -59,    43,  -140,  -140,  -140,   228,   -19,   205,  -140,
    -140,  -140,  -140,   -37,   -40,    67,  -140,   237,   -52,    77,
    -140,   227,   -41,   234,  -140,  -140,    -6,   156
};

  /* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     2,    10,    26,   209,   210,   211,   218,   222,    13,
      66,    67,    68,    88,   143,    89,    90,   203,   172,    14,
      82,   159,   132,   234,    15,    43,    44,    47,    77,   129,
      78,    16,    17,    38,    40,    18,    36,    37,    84,   134,
     133,    85,    86,    96,   123,   149,    60,    48
};

  /* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
     positive, shift that token.  If negative, reduce the rule whose
     number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_uint8 yytable[] =
{
       8,    57,    54,   171,    83,   185,    71,   117,     7,     1,
     115,   116,    76,   117,    39,    41,   118,    45,    98,     5,
      19,   115,   116,    91,   117,     7,     3,   118,    58,    73,
       7,     7,    53,    39,    58,    41,   119,   103,   101,   213,
      80,   214,     7,     7,    46,    58,   195,    75,   148,     7,
      79,     4,    58,     7,   122,     6,   135,   137,   154,     7,
       7,    56,   115,   142,   207,   117,    79,   136,   118,    39,
       9,    41,    95,   120,   117,    25,   144,    11,    12,   156,
     127,   161,   163,     7,    42,   165,    58,    46,   163,   163,
       7,    49,     7,    58,   128,    58,   130,    42,    61,   131,
      52,    91,    91,    27,    28,    29,    30,    31,    32,    63,
      64,    65,    33,    34,    55,    19,    69,   150,    35,   166,
     167,    46,    72,   155,    93,   192,   177,    20,    21,    74,
      22,   191,   193,   179,   157,   158,    81,   198,    23,    24,
      97,   160,    97,   162,    91,    64,    91,    97,   168,    97,
     169,   173,   174,   105,    63,    64,    50,   194,   206,   227,
     228,    87,    91,   105,   157,   236,   232,    92,   106,   107,
     108,   109,   110,   111,   223,   238,   157,   237,   106,   107,
     108,   109,   110,   111,   199,   112,   113,    97,     7,   140,
     141,    58,   100,   114,   104,   112,   113,   102,     7,   121,
     138,    58,   124,   107,   108,   109,   110,   125,    41,   126,
     139,   145,   190,   146,   147,   152,   220,   153,   164,   112,
     113,   170,     7,   176,   180,    58,    27,    28,    29,    30,
      31,    32,   181,   182,   183,    33,    34,   184,   186,   187,
     188,    35,   189,   200,   157,   194,   196,   213,   201,   202,
     204,   205,   208,   215,   217,   219,   221,   225,   226,   214,
     229,   230,   231,   235,   224,   216,   178,    94,   197,   175,
     233,    62,   212,    51,    70,     0,    59,    99,     0,   151
};

static const yytype_int16 yycheck[] =
{
       6,    41,    39,   142,    63,    34,    47,    36,    54,    58,
      33,    34,    58,    36,    20,    21,    39,    23,    70,    58,
       5,    33,    34,    64,    36,    54,     4,    39,    57,    48,
      54,    54,    56,    39,    57,    41,    59,    77,    75,    24,
      59,    26,    54,    54,    55,    57,   185,    53,    59,    54,
      56,     0,    57,    54,    95,     3,   108,   109,    59,    54,
      54,    56,    33,   115,   203,    36,    72,   108,    39,    75,
      59,    77,    36,    92,    36,    58,   117,    58,    59,   131,
      99,   133,   134,    54,    58,   137,    57,    55,   140,   141,
      54,     4,    54,    57,   100,    57,   102,    58,    59,   105,
      59,   142,   143,     6,     7,     8,     9,    10,    11,    30,
      31,    32,    15,    16,    59,     5,    58,   123,    21,   138,
     139,    55,    56,   129,    59,   184,   145,    17,    18,    58,
      20,   183,   184,   152,    58,    59,    58,   189,    28,    29,
      58,    59,    58,    59,   185,    31,   187,    58,    59,    58,
      59,    58,    59,    19,    30,    31,    59,    58,    59,    34,
      35,    58,   203,    19,    58,    59,   225,    58,    34,    35,
      36,    37,    38,    39,   214,   234,    58,    59,    34,    35,
      36,    37,    38,    39,   190,    51,    52,    58,    54,   112,
     113,    57,    23,    59,    59,    51,    52,    50,    54,    58,
      58,    57,    34,    35,    36,    37,    38,    39,   214,    59,
      58,    58,    27,    59,    59,    58,   212,    59,    59,    51,
      52,    58,    54,    59,    58,    57,     6,     7,     8,     9,
      10,    11,    59,    59,    59,    15,    16,    59,    59,    59,
      59,    21,    59,    59,    58,    58,    58,    24,    59,    59,
      59,    59,    58,    58,    58,    58,    58,    25,    59,    26,
      59,    59,    59,    59,   216,   209,   146,    67,   187,   143,
     227,    43,   205,    36,    47,    -1,    42,    72,    -1,   123
};

  /* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
     symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,    58,    62,     4,     0,    58,     3,    54,   107,    59,
      63,    58,    59,    70,    80,    85,    92,    93,    96,     5,
      17,    18,    20,    28,    29,    58,    64,     6,     7,     8,
       9,    10,    11,    15,    16,    21,    97,    98,    94,   107,
      95,   107,    58,    86,    87,   107,    55,    88,   108,     4,
      59,    98,    59,    56,    94,    59,    56,    95,    57,   104,
     107,    59,    87,    30,    31,    32,    71,    72,    73,    58,
     102,   103,    56,    88,    58,   107,    58,    89,    91,   107,
      88,    58,    81,    82,    99,   102,   103,    58,    74,    76,
      77,   103,    58,    59,    73,    36,   104,    58,    99,    89,
      23,    94,    50,    95,    59,    19,    34,    35,    36,    37,
      38,    39,    51,    52,    59,    33,    34,    36,    39,    59,
      88,    58,   103,   105,    34,    39,    59,    88,   107,    90,
     107,   107,    83,   101,   100,    99,   103,    99,    58,    58,
     100,   100,    99,    75,   103,    58,    59,    59,    59,   106,
     107,   108,    58,    59,    59,   107,    99,    58,    59,    82,
      59,    99,    59,    99,    59,    99,    88,    88,    59,    59,
      58,    77,    79,    58,    59,    76,    59,    88,    71,    88,
      58,    59,    59,    59,    59,    34,    59,    59,    59,    59,
      27,    99,    82,    99,    58,    77,    58,    74,    99,   107,
      59,    59,    59,    78,    59,    59,    59,    77,    58,    65,
      66,    67,    96,    24,    26,    58,    67,    58,    68,    58,
      66,    58,    69,    95,    68,    25,    59,    34,    35,    59,
      59,    59,    82,    83,    84,    59,    59,    59,    82
};

  /* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    61,    62,    63,    63,    63,    63,    63,    63,    63,
      64,    64,    65,    65,    65,    66,    67,    68,    69,    69,
      70,    70,    71,    71,    71,    71,    72,    72,    73,    73,
      74,    74,    75,    75,    76,    76,    76,    77,    77,    78,
      78,    79,    79,    80,    81,    81,    82,    82,    82,    83,
      83,    84,    84,    85,    86,    86,    87,    88,    88,    88,
      89,    89,    90,    90,    91,    92,    93,    94,    94,    94,
      95,    95,    95,    96,    97,    97,    98,    98,    98,    98,
      98,    98,    98,    98,    98,    99,    99,    99,    99,    99,
      99,    99,    99,    99,    99,   100,   100,   101,   101,   102,
     102,   103,   104,   104,   105,   105,   106,   106,   107,   108
};

  /* YYR2[YYN] -- Number of symbols on the right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     9,     0,     2,     2,     2,     2,     2,     2,
      14,    13,     1,     1,     2,     4,     4,     4,     4,     4,
       9,     5,     0,     1,     1,     2,     3,     2,     3,     2,
       1,     4,     0,     2,     7,     5,     1,     1,     4,     0,
       2,     1,     5,     6,     5,     1,     1,     4,     7,     0,
       2,     0,     2,     4,     1,     2,     4,     0,     4,     2,
       4,     1,     1,     2,     1,     4,     4,     0,     4,     2,
       0,     4,     2,     4,     1,     2,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     4,     4,     4,
       5,     7,     7,     4,     4,     0,     2,     0,     2,     1,
       4,     4,     1,     1,     0,     2,     1,     1,     1,     1
};


#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (yychar = YYEMPTY)
#define YYEMPTY         (-2)
#define YYEOF           0

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab


#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                                  \
do                                                              \
  if (yychar == YYEMPTY)                                        \
    {                                                           \
      yychar = (Token);                                         \
      yylval = (Value);                                         \
      YYPOPSTACK (yylen);                                       \
      yystate = *yyssp;                                         \
      goto yybackup;                                            \
    }                                                           \
  else                                                          \
    {                                                           \
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;                                                  \
    }                                                           \
while (0)

/* Error token number */
#define YYTERROR        1
#define YYERRCODE       256



/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)                        \
do {                                            \
  if (yydebug)                                  \
    YYFPRINTF Args;                             \
} while (0)

/* This macro is provided for backward compatibility. */
#ifndef YY_LOCATION_PRINT
# define YY_LOCATION_PRINT(File, Loc) ((void) 0)
#endif


# define YY_SYMBOL_PRINT(Title, Type, Value, Location)                    \
do {                                                                      \
  if (yydebug)                                                            \
    {                                                                     \
      YYFPRINTF (stderr, "%s ", Title);                                   \
      yy_symbol_print (stderr,                                            \
                  Type, Value); \
      YYFPRINTF (stderr, "\n");                                           \
    }                                                                     \
} while (0)


/*----------------------------------------.
| Print this symbol's value on YYOUTPUT.  |
`----------------------------------------*/

static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
{
  FILE *yyo = yyoutput;
  YYUSE (yyo);
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# endif
  YYUSE (yytype);
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
{
  YYFPRINTF (yyoutput, "%s %s (",
             yytype < YYNTOKENS ? "token" : "nterm", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

static void
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)                            \
do {                                                            \
  if (yydebug)                                                  \
    yy_stack_print ((Bottom), (Top));                           \
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

static void
yy_reduce_print (yytype_int16 *yyssp, YYSTYPE *yyvsp, int yyrule)
{
  unsigned long int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       yystos[yyssp[yyi + 1 - yynrhs]],
                       &(yyvsp[(yyi + 1) - (yynrhs)])
                                              );
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)          \
do {                                    \
  if (yydebug)                          \
    yy_reduce_print (yyssp, yyvsp, Rule); \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif


#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
static YYSIZE_T
yystrlen (const char *yystr)
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
static char *
yystpcpy (char *yydest, const char *yysrc)
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
        switch (*++yyp)
          {
          case '\'':
          case ',':
            goto do_not_strip_quotes;

          case '\\':
            if (*++yyp != '\\')
              goto do_not_strip_quotes;
            /* Fall through.  */
          default:
            if (yyres)
              yyres[yyn] = *yyp;
            yyn++;
            break;

          case '"':
            if (yyres)
              yyres[yyn] = '\0';
            return yyn;
          }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into *YYMSG, which is of size *YYMSG_ALLOC, an error message
   about the unexpected token YYTOKEN for the state stack whose top is
   YYSSP.

   Return 0 if *YYMSG was successfully written.  Return 1 if *YYMSG is
   not large enough to hold the message.  In that case, also set
   *YYMSG_ALLOC to the required number of bytes.  Return 2 if the
   required number of bytes is too large to store.  */
static int
yysyntax_error (YYSIZE_T *yymsg_alloc, char **yymsg,
                yytype_int16 *yyssp, int yytoken)
{
  YYSIZE_T yysize0 = yytnamerr (YY_NULLPTR, yytname[yytoken]);
  YYSIZE_T yysize = yysize0;
  enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
  /* Internationalized format string. */
  const char *yyformat = YY_NULLPTR;
  /* Arguments of yyformat. */
  char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
  /* Number of reported tokens (one for the "unexpected", one per
     "expected"). */
  int yycount = 0;

  /* There are many possibilities here to consider:
     - If this state is a consistent state with a default action, then
       the only way this function was invoked is if the default action
       is an error action.  In that case, don't check for expected
       tokens because there are none.
     - The only way there can be no lookahead present (in yychar) is if
       this state is a consistent state with a default action.  Thus,
       detecting the absence of a lookahead is sufficient to determine
       that there is no unexpected or expected token to report.  In that
       case, just report a simple "syntax error".
     - Don't assume there isn't a lookahead just because this state is a
       consistent state with a default action.  There might have been a
       previous inconsistent state, consistent state with a non-default
       action, or user semantic action that manipulated yychar.
     - Of course, the expected token list depends on states to have
       correct lookahead information, and it depends on the parser not
       to perform extra reductions after fetching a lookahead from the
       scanner and before detecting a syntax error.  Thus, state merging
       (from LALR or IELR) and default reductions corrupt the expected
       token list.  However, the list is correct for canonical LR with
       one exception: it will still contain any token that will not be
       accepted due to an error action in a later state.
  */
  if (yytoken != YYEMPTY)
    {
      int yyn = yypact[*yyssp];
      yyarg[yycount++] = yytname[yytoken];
      if (!yypact_value_is_default (yyn))
        {
          /* Start YYX at -YYN if negative to avoid negative indexes in
             YYCHECK.  In other words, skip the first -YYN actions for
             this state because they are default actions.  */
          int yyxbegin = yyn < 0 ? -yyn : 0;
          /* Stay within bounds of both yycheck and yytname.  */
          int yychecklim = YYLAST - yyn + 1;
          int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
          int yyx;

          for (yyx = yyxbegin; yyx < yyxend; ++yyx)
            if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR
                && !yytable_value_is_error (yytable[yyx + yyn]))
              {
                if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                  {
                    yycount = 1;
                    yysize = yysize0;
                    break;
                  }
                yyarg[yycount++] = yytname[yyx];
                {
                  YYSIZE_T yysize1 = yysize + yytnamerr (YY_NULLPTR, yytname[yyx]);
                  if (! (yysize <= yysize1
                         && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
                    return 2;
                  yysize = yysize1;
                }
              }
        }
    }

  switch (yycount)
    {
# define YYCASE_(N, S)                      \
      case N:                               \
        yyformat = S;                       \
      break
      YYCASE_(0, YY_("syntax error"));
      YYCASE_(1, YY_("syntax error, unexpected %s"));
      YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
      YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
      YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
      YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
# undef YYCASE_
    }

  {
    YYSIZE_T yysize1 = yysize + yystrlen (yyformat);
    if (! (yysize <= yysize1 && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
      return 2;
    yysize = yysize1;
  }

  if (*yymsg_alloc < yysize)
    {
      *yymsg_alloc = 2 * yysize;
      if (! (yysize <= *yymsg_alloc
             && *yymsg_alloc <= YYSTACK_ALLOC_MAXIMUM))
        *yymsg_alloc = YYSTACK_ALLOC_MAXIMUM;
      return 1;
    }

  /* Avoid sprintf, as that infringes on the user's name space.
     Don't have undefined behavior even if the translation
     produced a string with the wrong number of "%s"s.  */
  {
    char *yyp = *yymsg;
    int yyi = 0;
    while ((*yyp = *yyformat) != '\0')
      if (*yyp == '%' && yyformat[1] == 's' && yyi < yycount)
        {
          yyp += yytnamerr (yyp, yyarg[yyi++]);
          yyformat += 2;
        }
      else
        {
          yyp++;
          yyformat++;
        }
  }
  return 0;
}
#endif /* YYERROR_VERBOSE */

/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
{
  YYUSE (yyvaluep);
  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YYUSE (yytype);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}




/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;
/* Number of syntax errors so far.  */
int yynerrs;


/*----------.
| yyparse.  |
`----------*/

int
yyparse (void)
{
    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       'yyss': related to states.
       'yyvs': related to semantic values.

       Refer to the stacks through separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken = 0;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yyssp = yyss = yyssa;
  yyvsp = yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */
  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
        /* Give user a chance to reallocate the stack.  Use copies of
           these so that the &'s don't force the real ones into
           memory.  */
        YYSTYPE *yyvs1 = yyvs;
        yytype_int16 *yyss1 = yyss;

        /* Each stack pointer address is followed by the size of the
           data in use in that stack, in bytes.  This used to be a
           conditional around just the two extra args, but that might
           be undefined if yyoverflow is a macro.  */
        yyoverflow (YY_("memory exhausted"),
                    &yyss1, yysize * sizeof (*yyssp),
                    &yyvs1, yysize * sizeof (*yyvsp),
                    &yystacksize);

        yyss = yyss1;
        yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
        goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
        yystacksize = YYMAXDEPTH;

      {
        yytype_int16 *yyss1 = yyss;
        union yyalloc *yyptr =
          (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
        if (! yyptr)
          goto yyexhaustedlab;
        YYSTACK_RELOCATE (yyss_alloc, yyss);
        YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
        if (yyss1 != yyssa)
          YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
                  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
        YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = yylex ();
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
        goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     '$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 2:
#line 189 "lcp.y" /* yacc.c:1646  */
    {
 reader.m_domain_name = *(yyvsp[-4].str_val);
 reader.m_domain = *(yyvsp[-2].domain_);
 reader.m_problem = *(yyvsp[0].problem_);
}
#line 1491 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 3:
#line 198 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.domain_) = new pDomain;
}
#line 1499 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 4:
#line 203 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.domain_) = (yyvsp[-1].domain_);
  (yyval.domain_)->_require_def = *(yyvsp[0].requirement_);

}
#line 1509 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 5:
#line 210 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.domain_) = (yyvsp[-1].domain_);
  (yyval.domain_)->_types_def = *(yyvsp[0].typelist_);
}
#line 1518 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 6:
#line 216 "lcp.y" /* yacc.c:1646  */
    {
  //needed to check 
  (yyval.domain_) = (yyvsp[-1].domain_);
  (yyval.domain_)->_constants = *(yyvsp[0].termlist_);
}
#line 1528 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 7:
#line 223 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.domain_) = (yyvsp[-1].domain_);
  (yyval.domain_)->_predicates = *(yyvsp[0].atomlist_);
}
#line 1537 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 8:
#line 229 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.domain_) = (yyvsp[-1].domain_);
  (yyval.domain_)->_derived.push_back(*(yyvsp[0].structure_));
}
#line 1546 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 9:
#line 235 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.domain_) = (yyvsp[-1].domain_);
  (yyval.domain_)->_actions.push_back(*(yyvsp[0].structure_));
}
#line 1555 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 10:
#line 243 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.problem_) = (yyvsp[-3].problem_);
  (yyval.problem_)->_pname = *(yyvsp[-9].str_val);
  (yyval.problem_)->_dname = *(yyvsp[-5].str_val);
  (yyval.problem_)->_init = (yyvsp[-2].formula_);
  (yyval.problem_)->_goal = (yyvsp[-1].formula_);
  
}
#line 1568 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 11:
#line 253 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.problem_) = new pProblem;
  (yyval.problem_)->_pname = *(yyvsp[-8].str_val);
  (yyval.problem_)->_dname = *(yyvsp[-4].str_val);
  (yyval.problem_)->_init = (yyvsp[-2].formula_);
  (yyval.problem_)->_goal = (yyvsp[-1].formula_);
  
}
#line 1581 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 12:
#line 265 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.problem_) = new pProblem;
}
#line 1589 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 13:
#line 270 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.problem_) = new pProblem;
  (yyval.problem_)->_objects = *(yyvsp[0].termlist_);
}
#line 1598 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 14:
#line 276 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.problem_) = new pProblem;
  (yyval.problem_)->_objects = *(yyvsp[0].termlist_);
}
#line 1607 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 15:
#line 284 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.termlist_) = (yyvsp[-1].termlist_);
}
#line 1615 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 16:
#line 291 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 1623 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 17:
#line 298 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 1631 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 18:
#line 305 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 1639 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 19:
#line 310 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 1647 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 20:
#line 365 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.structure_) = (yyvsp[-1].structure_);
  (yyval.structure_)->n_type = DYNAMIC;
  (yyval.structure_)->_name = *(yyvsp[-6].str_val);
  (yyval.structure_)->_parameters = (yyvsp[-3].termlist_);	

}
#line 1659 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 21:
#line 374 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.structure_) = (yyvsp[-1].structure_);
  (yyval.structure_)->n_type = DYNAMIC;
  (yyval.structure_)->_name = *(yyvsp[-2].str_val);
  (yyval.structure_)->_parameters = new TermList;

}
#line 1671 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 22:
#line 384 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.structure_) = new Structure;
}
#line 1679 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 23:
#line 389 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.structure_) = new Structure;
  (yyval.structure_)->_preconditions = (yyvsp[0].formula_);
  (yyval.structure_)->_effects = 0;
}
#line 1689 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 24:
#line 396 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.structure_) = new Structure;
  (yyval.structure_)->_preconditions = 0;
  (yyval.structure_)->_effects = (yyvsp[0].formula_);
}
#line 1699 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 25:
#line 403 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.structure_) = new Structure;

  (yyval.structure_)->_preconditions = (yyvsp[-1].formula_);
  (yyval.structure_)->_effects = (yyvsp[0].formula_);

}
#line 1711 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 26:
#line 414 "lcp.y" /* yacc.c:1646  */
    {
  //empty precondition body
  (yyval.formula_) = new Formula;
}
#line 1720 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 27:
#line 420 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[0].formula_); 
}
#line 1728 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 28:
#line 428 "lcp.y" /* yacc.c:1646  */
    {
  //empty effect body
  (yyval.formula_) = new Formula;
}
#line 1737 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 29:
#line 434 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[0].formula_);
  //$2->print($2);
}
#line 1746 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 30:
#line 442 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[0].formula_);
}
#line 1754 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 31:
#line 447 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 1762 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 32:
#line 453 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = 0;
}
#line 1770 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 33:
#line 458 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = AND_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = (yyvsp[0].formula_);
}
#line 1781 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 34:
#line 468 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = QFORALL_;
  (yyval.formula_)->_quantified_parameters = *(yyvsp[-3].termlist_);
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = 0;
}
#line 1793 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 35:
#line 477 "lcp.y" /* yacc.c:1646  */
    {
 (yyval.formula_) = new Formula; 
 (yyval.formula_)->_operator = WHEN_;
 (yyval.formula_)->_t1 = (yyvsp[-2].formula_);
 (yyval.formula_)->_t2 = (yyvsp[-1].formula_);
}
#line 1804 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 36:
#line 485 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[0].formula_);
}
#line 1812 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 37:
#line 492 "lcp.y" /* yacc.c:1646  */
    {
  LiteralTerm* lt;
  lt = new LiteralTerm;
  lt->_atom = *(yyvsp[0].atom_);
  lt->_neg = false;

  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = UNIOP_;
  (yyval.formula_)->_term = *lt;
}
#line 1827 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 38:
#line 504 "lcp.y" /* yacc.c:1646  */
    {
 LiteralTerm* lt;
  lt = new LiteralTerm;
  lt->_atom = *(yyvsp[-1].atom_);
  lt->_neg = true;

  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = UNIOP_;
  (yyval.formula_)->_term = *lt;

}
#line 1843 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 39:
#line 518 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = 0;
}
#line 1851 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 40:
#line 523 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = AND_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = (yyvsp[0].formula_);
}
#line 1862 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 41:
#line 533 "lcp.y" /* yacc.c:1646  */
    {
 (yyval.formula_) = (yyvsp[0].formula_);
}
#line 1870 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 42:
#line 538 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = AND_;
  (yyval.formula_)->_t1 = (yyvsp[-2].formula_);
  (yyval.formula_)->_t2 = (yyvsp[-1].formula_);
}
#line 1881 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 43:
#line 548 "lcp.y" /* yacc.c:1646  */
    {
  LiteralList ll;
  LiteralList2 ll2; 
  Formula *f;

  (yyval.structure_) = new Structure;
  f = new Formula;

  f->_operator = UNIOP_;
  f->_term = *(yyvsp[-2].literal_);

  (yyval.structure_)->n_type = STATIC;
  (yyval.structure_)->_name = string("derived"+derivedname++);
  (yyval.structure_)->_parameters = (yyvsp[-3].termlist_);
  (yyval.structure_)->_preconditions = f;
  (yyval.structure_)->_effects = (yyvsp[-1].formula_);
}
#line 1903 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 44:
#line 570 "lcp.y" /* yacc.c:1646  */
    {
 //will be added
}
#line 1911 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 45:
#line 575 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[0].formula_);
}
#line 1919 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 46:
#line 583 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[0].formula_);
}
#line 1927 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 47:
#line 588 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 1935 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 48:
#line 593 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = QFORALL_;
  (yyval.formula_)->_quantified_parameters = *(yyvsp[-3].termlist_);
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = 0;
}
#line 1947 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 49:
#line 603 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = 0;
}
#line 1955 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 50:
#line 608 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = AND_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = (yyvsp[0].formula_);
}
#line 1966 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 51:
#line 617 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = 0;
}
#line 1974 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 52:
#line 622 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = OR_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = (yyvsp[0].formula_);
}
#line 1985 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 53:
#line 633 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.atomlist_) = (yyvsp[-1].atomlist_);
}
#line 1993 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 54:
#line 640 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.atomlist_) = new AtomList;
  (yyval.atomlist_)->push_back(*(yyvsp[0].atom_));
}
#line 2002 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 55:
#line 646 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.atomlist_) = (yyvsp[-1].atomlist_);
  (yyval.atomlist_)->push_back(*(yyvsp[0].atom_));
}
#line 2011 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 56:
#line 654 "lcp.y" /* yacc.c:1646  */
    {
  
  (yyval.atom_) = new Atom;
  (yyval.atom_)->_predicate = (yyvsp[-2].predicate_);
  (yyval.atom_)->_term = *(yyvsp[-1].termlist_);
}
#line 2022 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 57:
#line 663 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.termlist_) = new TermList;
}
#line 2030 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 58:
#line 668 "lcp.y" /* yacc.c:1646  */
    {
  map<string, string>::const_iterator p;

  TermList::iterator it;
  Term aterm;

  (yyval.termlist_) = (yyvsp[0].termlist_);
  aterm._name = *(yyvsp[-3].str_val);
  aterm._ground = *(yyvsp[-3].str_val);
  aterm._type = *(yyvsp[-1].typelist_);
  
  p = t_map.find((yyvsp[-1].typelist_)->front()._name);
  if (p != t_map.end()) {
     //aterm._type->front()._name = p->second;
	
     //keep the original type
     aterm._type.front()._name = p->first;
  }
  else {
   cout << "\nERROR: undeclared type " <<(yyvsp[-1].typelist_)->front()._name <<endl;
   return false;
  }
  
  aterm.constant = false;
  (yyval.termlist_)->push_front(aterm);
}
#line 2061 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 59:
#line 696 "lcp.y" /* yacc.c:1646  */
    {
  Term aterm, pterm;

  (yyval.termlist_) = (yyvsp[0].termlist_);
  aterm._name = *(yyvsp[-1].str_val);
  aterm._ground = *(yyvsp[-1].str_val);
  aterm.constant = true;
  if (!(yyvsp[0].termlist_)->empty()) {
   pterm = (yyvsp[0].termlist_)->front();
   if (!pterm.constant)
   { 
    aterm._type = pterm._type;
    aterm.constant = false;
   }
  }
  (yyval.termlist_)->push_front(aterm);
}
#line 2083 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 60:
#line 744 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.typelist_) = (yyvsp[-1].typelist_);
}
#line 2091 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 61:
#line 749 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.typelist_) = (yyvsp[0].typelist_);
}
#line 2099 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 62:
#line 756 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.typelist_) = new TypeList;
  Type nt;
  nt._name = *(yyvsp[0].str_val);
  (yyval.typelist_)->push_back(nt);
}
#line 2110 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 63:
#line 764 "lcp.y" /* yacc.c:1646  */
    {
  Type nt;
  (yyval.typelist_) = (yyvsp[-1].typelist_);
  nt._name = *(yyvsp[0].str_val);
  (yyval.typelist_)->push_back(nt);
}
#line 2121 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 64:
#line 774 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.typelist_) = new TypeList;
  Type nt;
  nt._name = *(yyvsp[0].str_val);
  (yyval.typelist_)->push_back(nt);
}
#line 2132 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 65:
#line 785 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.termlist_) = (yyvsp[-1].termlist_);
}
#line 2140 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 66:
#line 792 "lcp.y" /* yacc.c:1646  */
    {
  TypeList::iterator it;
  (yyval.typelist_) = (yyvsp[-1].typelist_);
  
  for (it = (yyvsp[-1].typelist_)->begin(); it != (yyvsp[-1].typelist_)->end(); it++) {
   //handle object type
   if (strupcase(it->_type) == "OBJECT")
     t_map.insert(value_type(it->_name, it->_name));
   else
     t_map.insert(value_type(it->_name, it->_type));
  }
}
#line 2157 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 67:
#line 807 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.typelist_) = new TypeList;
}
#line 2165 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 68:
#line 812 "lcp.y" /* yacc.c:1646  */
    {
  Type nt;
  
  (yyval.typelist_) = (yyvsp[0].typelist_);
  nt._name = *(yyvsp[-3].str_val);
  nt._type = *(yyvsp[-1].str_val);
  if (strupcase(*(yyvsp[-1].str_val)) == "OBJECT") //main type
    nt._type = "OBJECT";
  nt._typed = true;
  
  (yyval.typelist_)->push_front(nt); 
}
#line 2182 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 69:
#line 826 "lcp.y" /* yacc.c:1646  */
    {
  Type nt, pt;
 
  (yyval.typelist_) = (yyvsp[0].typelist_);
  nt._name = *(yyvsp[-1].str_val);
  nt._type = *(yyvsp[-1].str_val);
  nt._typed = false;
  if (!(yyvsp[0].typelist_)->empty()) {
   pt = (yyvsp[0].typelist_)->front();
   if (pt._typed) {
     nt._type = pt._type;
     nt._typed = true;
   }
  }
  (yyval.typelist_)->push_front(nt);
}
#line 2203 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 70:
#line 845 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.termlist_) = new TermList;
}
#line 2211 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 71:
#line 850 "lcp.y" /* yacc.c:1646  */
    {
  Term aterm;

  (yyval.termlist_) = (yyvsp[0].termlist_);
  aterm._name = *(yyvsp[-3].str_val);
  aterm._type = *(yyvsp[-1].typelist_);
  aterm.constant = false;
  (yyval.termlist_)->push_front(aterm);
  
}
#line 2226 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 72:
#line 862 "lcp.y" /* yacc.c:1646  */
    {
  Term aterm, pterm;

  (yyval.termlist_) = (yyvsp[0].termlist_);
  aterm._name = *(yyvsp[-1].str_val);
  aterm.constant = true;
  if (!(yyvsp[0].termlist_)->empty()) {
   pterm = (yyvsp[0].termlist_)->front();
   if (!pterm.constant)
   { 
    aterm._type = pterm._type;
    aterm.constant = false;
   }
  }
  (yyval.termlist_)->push_front(aterm);
}
#line 2247 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 73:
#line 882 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = (yyvsp[-1].requirement_); 
}
#line 2255 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 74:
#line 889 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = (yyvsp[0].requirement_);
}
#line 2263 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 75:
#line 894 "lcp.y" /* yacc.c:1646  */
    {
  StringList::iterator it;

  if ((yyvsp[-1].requirement_)->_require.empty())
    (yyval.requirement_) = new Requirement;
  else 
    (yyval.requirement_) = (yyvsp[-1].requirement_);
  for (it=(yyvsp[0].requirement_)->_require.begin();it!=(yyvsp[0].requirement_)->_require.end(); it++)
    (yyval.requirement_)->_require.insert(*it);
  
  //assign flags
  (yyval.requirement_)->adl 	= ((yyval.requirement_)->adl)||((yyvsp[0].requirement_)->adl);
  (yyval.requirement_)->strips 	= ((yyval.requirement_)->strips)||((yyvsp[0].requirement_)->strips);
  (yyval.requirement_)->equality 	= ((yyval.requirement_)->equality)||((yyvsp[0].requirement_)->equality);
  (yyval.requirement_)->typing 	= ((yyval.requirement_)->typing)||((yyvsp[0].requirement_)->typing);
  (yyval.requirement_)->negative_preconditions = ((yyval.requirement_)->negative_preconditions)||((yyvsp[0].requirement_)->negative_preconditions);
  (yyval.requirement_)->conditional_effects = ((yyval.requirement_)->conditional_effects)||((yyvsp[0].requirement_)->conditional_effects);
  (yyval.requirement_)->derived_predicates = ((yyval.requirement_)->derived_predicates)||((yyvsp[0].requirement_)->derived_predicates);
  (yyval.requirement_)->constraints = ((yyval.requirement_)->constraints)||((yyvsp[0].requirement_)->constraints);
  (yyval.requirement_)->disjunctive_preconditions = ((yyval.requirement_)->disjunctive_preconditions)||((yyvsp[0].requirement_)->disjunctive_preconditions);
}
#line 2289 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 76:
#line 919 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->strips = true;
}
#line 2299 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 77:
#line 926 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->adl = true;
}
#line 2309 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 78:
#line 933 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->equality = true;
}
#line 2319 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 79:
#line 940 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->typing = true;
}
#line 2329 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 80:
#line 947 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->disjunctive_preconditions = true;
}
#line 2339 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 81:
#line 954 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->negative_preconditions = true;
}
#line 2349 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 82:
#line 961 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->conditional_effects = true;

}
#line 2360 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 83:
#line 969 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->derived_predicates = true;

}
#line 2371 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 84:
#line 977 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.requirement_) = new Requirement;
  (yyval.requirement_)->_require.insert(*(yyvsp[0].str_val));
  (yyval.requirement_)->constraints = true;
  cout << "System has not support constraints"<<endl;
}
#line 2382 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 85:
#line 987 "lcp.y" /* yacc.c:1646  */
    {
  LiteralTerm* lt;
  lt = new LiteralTerm;
  lt->_atom = *(yyvsp[0].atom_);
  lt->_neg = false;

  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = UNIOP_;
  (yyval.formula_)->_term = *lt;
}
#line 2397 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 86:
#line 999 "lcp.y" /* yacc.c:1646  */
    { 
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = UNIOP_;
  (yyval.formula_)->_term = *(yyvsp[0].literal_);
}
#line 2407 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 87:
#line 1006 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 2415 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 88:
#line 1011 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = (yyvsp[-1].formula_);
}
#line 2423 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 89:
#line 1016 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = NOT_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = 0;
}
#line 2434 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 90:
#line 1024 "lcp.y" /* yacc.c:1646  */
    { //create node ~t1 v t2
  Formula* nott;
  nott = new Formula;
  nott->_operator = NOT_;
  nott->_t1 = (yyvsp[-2].formula_);
  nott->_t2 = 0;

  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = OR_;
  (yyval.formula_)->_t1 = nott;
  (yyval.formula_)->_t2 = (yyvsp[-1].formula_);
}
#line 2451 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 91:
#line 1038 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = QFORALL_;
  (yyval.formula_)->_quantified_parameters = *(yyvsp[-3].termlist_);
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = 0;
}
#line 2463 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 92:
#line 1047 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = QEXISTS_;
  (yyval.formula_)->_quantified_parameters = *(yyvsp[-3].termlist_);
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = 0;
}
#line 2475 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 93:
#line 1072 "lcp.y" /* yacc.c:1646  */
    {
  Formula *tmp, *node, *rnode;

  node = new Formula;
  rnode = new Formula;

  copy_formula((yyvsp[-1].formula_), node);

  rnode->_operator = NOT_;
  rnode->_t1=node;
  rnode->_t2=0;
  negation_movein(rnode);

  tmp = new Formula;
  tmp->_operator = OR_;
  tmp->_t1 = (yyvsp[-1].formula_);
  tmp->_t2 = rnode;

  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = ONEOF_;
  (yyval.formula_)->_t1 = tmp;
  (yyval.formula_)->_t2 = 0;
}
#line 2503 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 94:
#line 1099 "lcp.y" /* yacc.c:1646  */
    { 

  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = ONEOF_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = 0;
/*
  //To robust parsing ONEOF laterly, NOT_ and OR node are construct 
  Formula *nNode, *f;
  int nl; 

  nNode = new Formula;
  nNode->_operator = NOT_;
  nNode->_t1 = $3;
  nNode->_t2 = 0;

  f = new Formula;
  f->_operator = ONEOF_;
  f->_t1 = new Formula;

  negation_movein(nNode);
  nl = totaltreeleaves(nNode);
  addtree(f->_t1, nNode, nl, nl);
  $$ = f->_t1;
  $$->print();
*/
}
#line 2535 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 95:
#line 1130 "lcp.y" /* yacc.c:1646  */
    { 
  (yyval.formula_) = 0;
}
#line 2543 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 96:
#line 1135 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = OR_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = (yyvsp[0].formula_);
}
#line 2554 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 97:
#line 1145 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = 0;
}
#line 2562 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 98:
#line 1150 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.formula_) = new Formula;
  (yyval.formula_)->_operator = AND_;
  (yyval.formula_)->_t1 = (yyvsp[-1].formula_);
  (yyval.formula_)->_t2 = (yyvsp[0].formula_);
}
#line 2573 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 99:
#line 1175 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.literal_) = new LiteralTerm;
  (yyval.literal_)->_atom = *(yyvsp[0].atom_);
  (yyval.literal_)->_neg = false;
}
#line 2583 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 100:
#line 1182 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.literal_) = new LiteralTerm;
  (yyval.literal_)->_atom = *(yyvsp[-1].atom_);
  (yyval.literal_)->_neg = true; 
}
#line 2593 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 101:
#line 1191 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.atom_) = new Atom;
  (yyval.atom_)->_predicate = (yyvsp[-2].predicate_);
  (yyval.atom_)->_term = *(yyvsp[-1].termlist_);

}
#line 2604 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 102:
#line 1201 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.predicate_) = new Predicate;
  (yyval.predicate_)->_name = *(yyvsp[0].str_val);
}
#line 2613 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 103:
#line 1207 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.predicate_) = new Predicate;
  (yyval.predicate_)->_name = "equal";
}
#line 2622 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 104:
#line 1214 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.termlist_) = new TermList;
}
#line 2630 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 105:
#line 1219 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.termlist_) = (yyvsp[-1].termlist_);
  (yyval.termlist_)->push_back(*(yyvsp[0].term_));
}
#line 2639 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 106:
#line 1227 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.term_) = new Term;
  (yyval.term_)->_name = *(yyvsp[0].str_val);
  (yyval.term_)->_ground = *(yyvsp[0].str_val); //and 04/05/06
  (yyval.term_)->constant = true; 
  (yyval.term_)->variable = false;
}
#line 2651 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 107:
#line 1236 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.term_) = new Term;
  (yyval.term_)->_name = *(yyvsp[0].str_val);
  (yyval.term_)->_ground = *(yyvsp[0].str_val); //and 04/05/06
  (yyval.term_)->variable = true; 
  (yyval.term_)->constant = false;
}
#line 2663 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 108:
#line 1246 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.str_val) = (yyvsp[0].str_val);
}
#line 2671 "lcp.tab.c" /* yacc.c:1646  */
    break;

  case 109:
#line 1259 "lcp.y" /* yacc.c:1646  */
    {
  (yyval.str_val) = (yyvsp[0].str_val);
}
#line 2679 "lcp.tab.c" /* yacc.c:1646  */
    break;


#line 2683 "lcp.tab.c" /* yacc.c:1646  */
      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;

  /* Now 'shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*--------------------------------------.
| yyerrlab -- here on detecting error.  |
`--------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYEMPTY : YYTRANSLATE (yychar);

  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
# define YYSYNTAX_ERROR yysyntax_error (&yymsg_alloc, &yymsg, \
                                        yyssp, yytoken)
      {
        char const *yymsgp = YY_("syntax error");
        int yysyntax_error_status;
        yysyntax_error_status = YYSYNTAX_ERROR;
        if (yysyntax_error_status == 0)
          yymsgp = yymsg;
        else if (yysyntax_error_status == 1)
          {
            if (yymsg != yymsgbuf)
              YYSTACK_FREE (yymsg);
            yymsg = (char *) YYSTACK_ALLOC (yymsg_alloc);
            if (!yymsg)
              {
                yymsg = yymsgbuf;
                yymsg_alloc = sizeof yymsgbuf;
                yysyntax_error_status = 2;
              }
            else
              {
                yysyntax_error_status = YYSYNTAX_ERROR;
                yymsgp = yymsg;
              }
          }
        yyerror (yymsgp);
        if (yysyntax_error_status == 2)
          goto yyexhaustedlab;
      }
# undef YYSYNTAX_ERROR
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
         error, discard it.  */

      if (yychar <= YYEOF)
        {
          /* Return failure if at end of input.  */
          if (yychar == YYEOF)
            YYABORT;
        }
      else
        {
          yydestruct ("Error: discarding",
                      yytoken, &yylval);
          yychar = YYEMPTY;
        }
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule whose action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;      /* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
        {
          yyn += YYTERROR;
          if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
            {
              yyn = yytable[yyn];
              if (0 < yyn)
                break;
            }
        }

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
        YYABORT;


      yydestruct ("Error: popping",
                  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined yyoverflow || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval);
    }
  /* Do not reclaim the symbols of the rule whose action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
                  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  return yyresult;
}
#line 1264 "lcp.y" /* yacc.c:1906  */


int yyerror(string s)
{
  extern int yylineno;	// defined and maintained in lex.c
  extern char *yytext;	// defined and maintained in lex.c
  
  cerr << "ERROR: " << s << " at symbol \"" << yytext;
  cerr << "\" on line " << yylineno << endl;
  exit(1);
  return 0;  
}

int yyerror(char *s)
{
  return yyerror(string(s));
}


int totaltreeleaves(Formula *node) 
{
  int nleaves = 0;

  switch (node->_operator) {
    case AND_:
      if (node->_t1 != 0) nleaves = nleaves + totaltreeleaves(node->_t1);
      if (node->_t2 != 0) nleaves = nleaves + totaltreeleaves(node->_t2);
      break;
    case UNIOP_:
      nleaves++;
      break;
    default:
      printf("Error in oneof %d syntax! \n",node->_operator);
      break;
  }
  return (nleaves);
}

int ranktree(Formula *node, int totalrank, int pos) 
{
  static int rank = 0;
  switch (node->_operator) {
    case ONEOF_:
    case AND_:
      if (node->_t1 != 0) ranktree(node->_t1,totalrank,pos);
      if (node->_t2 != 0) ranktree(node->_t2,totalrank,pos);
      break;
    case UNIOP_:
      rank++;
      if (rank == pos)
         node->_term._neg = false; 
      if (rank == totalrank) rank = 0;
      break;
    default:
      printf("Error in oneof %d syntax! \n",node->_operator);
      break;
  }
  return rank;
}

void negation_movein(Formula* f)
{ 
  switch (f->_operator) {
   case AND_:
   case OR_:
   case IMPLY_:
   case QFORALL_:
   case QEXISTS_: 
   case UNIOP_:
   case WHEN_:
   case ONEOF_:
	if (f->_t1 != 0) negation_movein(f->_t1);
	if (f->_t2 != 0) negation_movein(f->_t2);
        break;
   case NOT_: 
	//shift tree up one level as negating the NOT
        if (f->_t1 != 0 ) {
	f->_operator = f->_t1->_operator;
	f->_term = f->_t1->_term;
	f->_quantified_parameters = f->_t1->_quantified_parameters;
	f->_t2 = f->_t1->_t2;
	f->_t1 = f->_t1->_t1;	
	f->negation();
        }
	break;
   default: 
        break;
  }
}

void addtree(Formula *f, Formula *node, int totalrank, int i)
{
   Formula *cpnode;

     cpnode = new Formula;
     copy_formula(node, cpnode);
     ranktree(cpnode,totalrank,i);

     f->_operator = OR_;
     f->_t1 = cpnode;
     f->_t2 = new Formula;
     if (--i>0) 
       addtree(f->_t2,node, totalrank, i);
}

void copy_formula(Formula *f, Formula *cp)
//cp is a copied formula of the pointer formula f
{
    cp->_operator = f->_operator;
    cp->_quantified_parameters = f->_quantified_parameters;
    cp->_term = f->_term;
    if (f->_t1 != 0) { 
      cp->_t1 = new Formula;
      copy_formula (f->_t1, cp->_t1);
    }
    if (f->_t2 != 0) { 
      cp->_t2 = new Formula;
      copy_formula (f->_t2, cp->_t2);
    }
}

string strupcase(const string& str )
{
   string tmp=str;

   for(int i = 0; i < str.length(); i++)
   {
      tmp[i] = toupper(str[i]);
   }
   return tmp;
}  

