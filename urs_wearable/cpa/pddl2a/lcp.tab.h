/* A Bison parser, made by GNU Bison 3.0.4.  */

/* Bison interface for Yacc-like parsers in C

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
#line 23 "lcp.y" /* yacc.c:1909  */

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

#line 137 "lcp.tab.h" /* yacc.c:1909  */
};

typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE yylval;

int yyparse (void);

#endif /* !YY_YY_LCP_TAB_H_INCLUDED  */
