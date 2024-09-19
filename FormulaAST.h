#pragma once

#include "FormulaLexer.h"
#include "common.h"

#include <forward_list>
#include <functional>
#include <optional>
#include <stdexcept>

namespace ASTImpl {
	class Expr;
}

class ParsingError : public std::runtime_error {
	using std::runtime_error::runtime_error;
};

using CellFunction = std::optional<std::function<double(const Position&)>>;

class FormulaAST {
public:
	explicit FormulaAST(std::unique_ptr<ASTImpl::Expr> root_expr, std::forward_list<Position> cells);
	FormulaAST(FormulaAST&&) = default;
	FormulaAST& operator=(FormulaAST&&) = default;
	~FormulaAST();

	double Execute(CellFunction cellfunc) const;
	void Print(std::ostream& out) const;
	void PrintFormula(std::ostream& out) const;
	const std::forward_list<Position>& GetCells() const;

private:
	std::unique_ptr<ASTImpl::Expr> root_expr_;
	std::forward_list<Position> cells_;
};

FormulaAST ParseFormulaAST(std::istream& in);
FormulaAST ParseFormulaAST(const std::string& in_str);